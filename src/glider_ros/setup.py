from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'glider_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='james@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_bno085_node = glider_ros.drivers.imu_bno085_node:main',
            'gnss_maxm10s_i2c_node = glider_ros.drivers.gnss_maxm10s_i2c_node:main',
            'communication_iridium_node = glider_ros.drivers.communication_iridium_node:main',
            'sonar_ping_node = glider_ros.drivers.sonar_ping_node:main',
            'can_bridge_node = glider_ros.manager.can_bridge_node:main',
            'pressure_keller_node = glider_ros.drivers.pressure_keller_node:main',
            'safety_node = glider_ros.safety.safety_node:main',
            'telemetry_manager_node = glider_ros.manager.telemetry_manager_node:main',
            'iridium_signal_node = glider_ros.tests.iridium_signal_node:main',
            'bridge_catch_iridium_node = glider_ros.tests.bridge_catch_iridium_node:main',
            'state_manager_node = glider_ros.manager.state_manager_node:main',
            'imu_manager_node = glider_ros.manager.imu_manager_node:main',
            'sonar_manager_node = glider_ros.manager.sonar_manager_node:main',
            'glider_controller_node = glider_ros.controllers.controller_node:main',
            'controller_node = glider_ros.controllers.controller_node:main',
        ],
    },
)