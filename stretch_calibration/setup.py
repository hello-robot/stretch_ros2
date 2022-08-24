from setuptools import setup, find_packages
from glob import glob

package_name = 'stretch_calibration'

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
    description='The stretch calibration package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collect_head_calibration_data = stretch_calibration.collect_head_calibration_data:main',
            'process_head_calibration_data = stretch_calibration.process_head_calibration_data:main',
            'check_head_calibration = stretch_calibration.check_head_calibration:main', 
        ],
    },
)
