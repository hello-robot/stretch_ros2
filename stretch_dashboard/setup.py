from setuptools import find_packages, setup

package_name = 'stretch_dashboard'
setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    license='Apache License 2.0',
    author='David V. Lu!!',
    author_email='davidvlu@gmail.com',
    description='A Python GUI plugin for displaying a dashboard that displays and interacts with the Stretch robot.',
    entry_points={
        'console_scripts': [
            'dashboard = stretch_dashboard.main:main',
        ],
    },
)
