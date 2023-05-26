from setuptools import setup, find_packages
from glob import glob

package_name = 'stretch_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    url='https://github.com/hello-robot/stretch_ros2',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The stretch demos package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clean_surface = stretch_demos.clean_surface:main'
        ],
    },
)
