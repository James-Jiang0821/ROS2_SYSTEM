import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class GliderMarkerNode(Node):
    def __init__(self):
        super().__init__('glider_marker_node')

        self.declare_parameter('pose_topic', '/model/glider/pose')
        self.declare_parameter('marker_topic', '/ug/vis/glider_marker')
        self.declare_parameter('frame_id', 'world')  # safest default
        self.declare_parameter('scale', 1.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scale = float(self.get_parameter('scale').value)

        self.latest_pose = None

        self.sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_cb,
            10
        )

        self.pub = self.create_publisher(Marker, self.marker_topic, 10)

        # Publish at a steady rate (RViz likes this)
        self.timer = self.create_timer(0.05, self.publish_marker)  # 20 Hz

        self.get_logger().info(f"Subscribing: {self.pose_topic}")
        self.get_logger().info(f"Publishing:  {self.marker_topic}")

    def pose_cb(self, msg: PoseStamped):
        self.latest_pose = msg

    def publish_marker(self):
        if self.latest_pose is None:
            return

        m = Marker()

        # IMPORTANT anti-flicker: stamp = 0 means "latest transform"
        m.header.stamp.sec = 0
        m.header.stamp.nanosec = 0

        # Use a fixed world-like frame to avoid TF dependency
        m.header.frame_id = self.frame_id

        m.ns = "glider"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Put the cube at the pose position
        m.pose.position = self.latest_pose.pose.position
        m.pose.orientation = self.latest_pose.pose.orientation

        m.scale.x = self.scale
        m.scale.y = self.scale
        m.scale.z = self.scale

        # Infinite lifetime
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0

        # Leave colour defaults if you want; RViz will still show it if set.
        # But many RViz setups require alpha > 0:
        m.color.a = 1.0
        m.color.r = 0.2
        m.color.g = 0.8
        m.color.b = 1.0

        self.pub.publish(m)


def main():
    rclpy.init()
    node = GliderMarkerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
