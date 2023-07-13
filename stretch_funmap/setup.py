from setuptools import setup, find_packages
from Cython.Build import cythonize
from glob import glob

package_name = 'stretch_funmap'
cython_files = [package_name+"/cython_min_cost_path.pyx",]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    # package_data={'stretch_demos': ['*.pyx']},
    ext_modules = cythonize(cython_files, compiler_directives={'language_level': "3"}, force=True, quiet=True),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools', 'wheel', 'Cython'],
    url='https://github.com/hello-robot/stretch_ros2',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The stretch funmap package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'funmap = stretch_funmap.funmap:main'
        ],
    },
)
