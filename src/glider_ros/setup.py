from setuptools import find_packages, setup

package_name = 'glider_ros'

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
            'bno085_imu_node = glider_ros.drivers.bno085_imu_node:main',
            'gnss_maxm10s_i2c = glider_ros.drivers.gnss_maxm10s_i2c_node:main',
            'iridium_basic_node = glider_ros.drivers.iridium_basic_node:main',
        ],
    },
)
