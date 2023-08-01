from setuptools import setup, find_packages
from Cython.Build import cythonize
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
    install_requires=['setuptools', 'wheel'],
    url='https://github.com/hello-robot/stretch_ros2',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The stretch demos package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clean_surface = stretch_demos.clean_surface:main',
            'open_drawer = stretch_demos.open_drawer:main',
            'grasp_object = stretch_demos.grasp_object:main',
            'handover_object = stretch_demos.handover_object:main',
            'hello_world = stretch_demos.hello_world:main'
        ],
    },
)
