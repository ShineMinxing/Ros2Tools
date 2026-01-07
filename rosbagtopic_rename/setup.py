from setuptools import setup

package_name = 'rosbagtopic_rename'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='you@example.com',
    description='Offline tool to rename topics inside a rosbag2 bag.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run rosbagtopic_rename rosbagtopic_rename_node
            'rosbagtopic_rename_node = rosbagtopic_rename.rosbagtopic_rename_node:main',
        ],
    },
)
