from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'basicmicro_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yml')),
        # Include URDF files (when added)
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
        # Include service files
        (os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'geometry_msgs', 
        'sensor_msgs',
        'nav_msgs',
        'diagnostic_msgs',
        'control_msgs',
        'pyserial',
        'numpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 driver for Basicmicro (formerly RoboClaw) motor controllers using ros2_control framework',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basicmicro_node = basicmicro_driver.basicmicro_node:main',
            'basicmicro_test = basicmicro_driver.test_node:main',
            'motion_config_service_node = basicmicro_driver.motion_config_service:main',
            'distance_movement_service_node = basicmicro_driver.distance_movement_service:main',
            'trajectory_service_node = basicmicro_driver.trajectory_service:main',
            'duty_control_service_node = basicmicro_driver.duty_control_service:main',
            'servo_position_service_node = basicmicro_driver.servo_position_service:main',
            'arm_node = basicmicro_driver.arm_node:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering :: Robotics',
    ],
    python_requires='>=3.8',
)