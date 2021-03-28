from setuptools import setup

package_name = 'ds_ros2_drone_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rostest1',
    maintainer_email='martin.hermansen@hotmail.com',
    description='Python package on for PX4 to ROS2 communication.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'px4_offboard_control = ds_ros2_drone_pkg.px4_offboard_control:main',
        ],
    },
)
