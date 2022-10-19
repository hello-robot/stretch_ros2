from setuptools import setup
from glob import glob

package_name = 'stretch_deep_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hello Robot Inc.',
    maintainer_email='support@hello-robot.com',
    description='The stretch_deep_perception package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_objects = stretch_deep_perception.detect_objects:main',
            'detect_faces = stretch_deep_perception.detect_faces:main',
        ],
    },
)
