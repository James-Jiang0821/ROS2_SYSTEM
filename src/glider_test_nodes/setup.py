from setuptools import find_packages, setup

package_name = 'glider_test_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'fake_leak_publisher = glider_test_nodes.fake_leak_publisher:main',
            'imu_leak_monitor = glider_test_nodes.imu_leak_monitor:main',
            'glider_marker_node = glider_test_nodes.glider_marker_node:main',

        ],
    },
)
