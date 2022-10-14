from setuptools import setup, find_packages
from glob import glob

package_name = 'stretch_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    url='https://github.com/hello-robot/stretch_ros2',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The stretch_core package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stretch_driver = stretch_core.stretch_driver:main',
            'detect_aruco_markers = stretch_core.detect_aruco_markers:main',
            'd435i_accel_correction = stretch_core.d435i_accel_correction:main',
            'keyboard_teleop = stretch_core.keyboard_teleop:main',
            'avoider = stretch_core.avoider:main',
            'align_to_aruco = stretch_core.align_to_aruco:main',
        ],
    },
)
