from setuptools import find_packages, setup
from glob import glob

package_name = 'stretch_mujoco_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scene', glob('scene/*.xml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='xmo@cs.wisc.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stretch_sim_driver = stretch_mujoco_ros2.stretch_sim_driver:main',
        ],
    },
)
