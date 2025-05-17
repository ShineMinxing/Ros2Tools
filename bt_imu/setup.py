from setuptools import setup

package_name = 'bt_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'bleak',
        'rclpy',
        'sensor_msgs',
    ],
    zip_safe=True,
    author='your_name',
    author_email='your_email@example.com',
    description='ROS2 驱动 BWT901BLE5.0 IMU 的 Python 节点',
    license='Apache License 2.0',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',
    ],
    entry_points={
        'console_scripts': [
            'bt_imu_node = bt_imu.bt_imu_node:main',
        ],
    },
)
