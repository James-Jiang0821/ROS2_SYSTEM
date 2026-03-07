#!/usr/bin/env python3
import time
import struct
from collections import deque

from smbus import SMBus

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


UBX_SYNC_1 = 0xB5
UBX_SYNC_2 = 0x62

UBX_CLASS_NAV = 0x01
UBX_ID_NAV_PVT = 0x07  # main PVT solution

# u-blox DDC/I2C registers (MAX-M10S)
REG_AVAIL_L = 0xFD
REG_AVAIL_H = 0xFE
REG_STREAM  = 0xFF


def ubx_checksum(payload: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


class UbxStreamParser:
    """
    Minimal UBX framing parser.
    Feed bytes; yields complete UBX messages: (cls, id, payload_bytes).
    """
    def __init__(self, max_payload=2048):
        self.buf = deque()
        self.max_payload = max_payload

    def feed(self, data: bytes):
        for b in data:
            self.buf.append(b)

    def _pop(self) -> int:
        return self.buf.popleft()

    def _peek(self, i: int) -> int:
        # i=0 is first element
        return list(self.buf)[i]

    def messages(self):
        msgs = []
        while True:
            # need at least sync + class+id+len2 + ck2
            if len(self.buf) < 8:
                break

            # hunt sync
            if self._peek(0) != UBX_SYNC_1 or self._peek(1) != UBX_SYNC_2:
                self._pop()
                continue

            # header available?
            if len(self.buf) < 6:
                break

            # read header without removing yet
            cls_ = self._peek(2)
            id_ = self._peek(3)
            length = self._peek(4) | (self._peek(5) << 8)
            if length > self.max_payload:
                # desync protection
                self._pop()
                continue

            total_len = 2 + 4 + length + 2  # sync(2) + cls/id/len(4) + payload + ck(2)
            if len(self.buf) < total_len:
                break

            # now we can pop the message
            sync1 = self._pop(); sync2 = self._pop()
            cls_b = self._pop()
            id_b = self._pop()
            len_l = self._pop()
            len_h = self._pop()
            payload = bytes(self._pop() for _ in range(length))
            ck_a = self._pop()
            ck_b = self._pop()

            # checksum is over: cls,id,len_l,len_h,payload
            ck_payload = bytes([cls_b, id_b, len_l, len_h]) + payload
            exp_a, exp_b = ubx_checksum(ck_payload)
            if (ck_a, ck_b) != (exp_a, exp_b):
                # checksum fail -> resync by discarding one byte and continuing
                # (we already consumed; safest is continue)
                continue

            msgs.append((cls_b, id_b, payload))

        return msgs


def parse_nav_pvt(payload: bytes):
    """
    Parse UBX-NAV-PVT payload (92 bytes).
    Returns dict of useful fields or None if wrong length.
    """
    if len(payload) < 92:
        return None

    # Little-endian; layout per u-blox UBX NAV-PVT definition (M8/M9/M10 very similar)
    # We'll unpack only what we need (by offsets).
    iTOW = struct.unpack_from("<I", payload, 0)[0]
    year = struct.unpack_from("<H", payload, 4)[0]
    month = payload[6]
    day = payload[7]
    hour = payload[8]
    minute = payload[9]
    sec = payload[10]
    fix_type = payload[20]
    flags = payload[21]
    num_sv = payload[23]

    lon = struct.unpack_from("<i", payload, 24)[0] * 1e-7  # deg
    lat = struct.unpack_from("<i", payload, 28)[0] * 1e-7  # deg
    height_msl_mm = struct.unpack_from("<i", payload, 36)[0]  # mm (hMSL)
    h_acc_mm = struct.unpack_from("<I", payload, 40)[0]       # mm
    v_acc_mm = struct.unpack_from("<I", payload, 44)[0]       # mm

    vel_n_mmps = struct.unpack_from("<i", payload, 48)[0]
    vel_e_mmps = struct.unpack_from("<i", payload, 52)[0]
    vel_d_mmps = struct.unpack_from("<i", payload, 56)[0]
    gspeed_mmps = struct.unpack_from("<i", payload, 60)[0]
    head_mot_1e5 = struct.unpack_from("<i", payload, 64)[0]   # 1e-5 deg

    # flags: bit0 gnssFixOK; bit1 diffSoln; etc.
    gnss_fix_ok = bool(flags & 0x01)

    return {
        "iTOW": iTOW,
        "datetime": (year, month, day, hour, minute, sec),
        "fix_type": fix_type,
        "gnss_fix_ok": gnss_fix_ok,
        "num_sv": num_sv,
        "lat_deg": lat,
        "lon_deg": lon,
        "alt_m": height_msl_mm / 1000.0,
        "h_acc_m": h_acc_mm / 1000.0,
        "v_acc_m": v_acc_mm / 1000.0,
        "vel_n_mps": vel_n_mmps / 1000.0,
        "vel_e_mps": vel_e_mmps / 1000.0,
        "vel_d_mps": vel_d_mmps / 1000.0,
        "ground_speed_mps": gspeed_mmps / 1000.0,
        "head_mot_deg": head_mot_1e5 * 1e-5,
    }


class MaxM10sI2CNode(Node):
    def __init__(self):
        super().__init__("gnss_maxm10s_i2c")

        # Params
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", 0x42)
        self.declare_parameter("poll_hz", 2.0)          # poll stream
        self.declare_parameter("max_read_bytes", 256)    # per I2C transaction
        self.declare_parameter("publish_vel", True)

        self.i2c_bus_num = int(self.get_parameter("i2c_bus").value)
        self.addr = int(self.get_parameter("i2c_addr").value)
        self.poll_hz = float(self.get_parameter("poll_hz").value)
        self.max_read = int(self.get_parameter("max_read_bytes").value)
        self.publish_vel = bool(self.get_parameter("publish_vel").value)

        # Publishers
        self.pub_fix = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.pub_vel = self.create_publisher(TwistStamped, "/gps/vel", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/gps/diagnostics", 10)

        self.parser = UbxStreamParser()
        self.last_pvt_time = None
        self.last_diag_pub = 0.0

        # I2C bus
        self.bus = SMBus(self.i2c_bus_num)

        period = 1.0 / max(self.poll_hz, 1e-3)
        self.timer = self.create_timer(period, self.poll_once)

        self.get_logger().info(
            f"MAX-M10S I2C GNSS started on /dev/i2c-{self.i2c_bus_num}, addr 0x{self.addr:02X}"
        )

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()

    def _read_avail(self) -> int:
        lo = self.bus.read_byte_data(self.addr, REG_AVAIL_L)
        hi = self.bus.read_byte_data(self.addr, REG_AVAIL_H)
        return (lo << 8) | hi

    def _read_stream(self, nbytes: int) -> bytes:
        n = min(nbytes, self.max_read)
        data = bytearray()

        for _ in range(n):
            data.append(self.bus.read_byte_data(self.addr, REG_STREAM))

        return bytes(data)

    def poll_once(self):
        try:
            avail = self._read_avail()
            self.get_logger().info(f"GNSS available bytes: {avail}")

            if avail == 0:
                self._publish_diag_if_needed(avail=0)
                return

            # read in chunks
            remaining = avail
            while remaining > 0:
                chunk = self._read_stream(remaining)
                if not chunk:
                    break
                remaining -= len(chunk)

                # u-blox notes that 0xFF can return 0xFF when no data is pending :contentReference[oaicite:2]{index=2}
                # We'll still feed it; parser will ignore unless it forms a valid UBX frame.
                self.parser.feed(chunk)

                for cls_, id_, payload in self.parser.messages():
                    if cls_ == UBX_CLASS_NAV and id_ == UBX_ID_NAV_PVT:
                        pvt = parse_nav_pvt(payload)
                        if pvt:
                            self._publish_pvt(pvt)

            self._publish_diag_if_needed(avail=avail)

        except OSError as e:
            self.get_logger().warn(f"I2C read error: {e}")
        except Exception as e:
            self.get_logger().warn(f"GNSS node exception: {e}")

    def _publish_pvt(self, pvt: dict):
        now = self.get_clock().now().to_msg()
        self.last_pvt_time = time.time()

        # NavSatFix
        msg = NavSatFix()
        msg.header.stamp = now
        msg.header.frame_id = "gps_link"  # you can standardize later

        msg.status.service = NavSatStatus.SERVICE_GPS  # conservative
        if pvt["gnss_fix_ok"] and pvt["fix_type"] >= 3:
            msg.status.status = NavSatStatus.STATUS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        msg.latitude = float(pvt["lat_deg"])
        msg.longitude = float(pvt["lon_deg"])
        msg.altitude = float(pvt["alt_m"])

        # Covariance (very rough): use hAcc/vAcc as 1-sigma, convert to variance
        h = max(pvt["h_acc_m"], 0.0)
        v = max(pvt["v_acc_m"], 0.0)
        msg.position_covariance = [
            h*h, 0.0, 0.0,
            0.0, h*h, 0.0,
            0.0, 0.0, v*v
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_fix.publish(msg)

        # Velocity (ENU)
        if self.publish_vel:
            vel = TwistStamped()
            vel.header.stamp = now
            vel.header.frame_id = "gps_link"
            vel.twist.linear.x = float(pvt["vel_e_mps"])
            vel.twist.linear.y = float(pvt["vel_n_mps"])
            vel.twist.linear.z = float(-pvt["vel_d_mps"])  # down -> up
            self.pub_vel.publish(vel)

    def _publish_diag_if_needed(self, avail: int):
        # publish diag at ~1 Hz
        t = time.time()
        if (t - self.last_diag_pub) < 1.0:
            return
        self.last_diag_pub = t

        age = (t - self.last_pvt_time) if self.last_pvt_time else float("inf")

        st = DiagnosticStatus()
        st.level = DiagnosticStatus.OK if age < 2.0 else DiagnosticStatus.WARN
        st.name = "gnss_maxm10s_i2c"
        st.message = "OK" if age < 2.0 else "No recent NAV-PVT"
        st.values = [
            KeyValue(key="i2c_addr", value=f"0x{self.addr:02X}"),
            KeyValue(key="avail_bytes_last_poll", value=str(avail)),
            KeyValue(key="age_s_since_last_pvt", value=f"{age:.2f}"),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)


def main():
    rclpy.init()
    node = MaxM10sI2CNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()