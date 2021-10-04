from setuptools import setup, find_packages

package_name = 'stretch_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    url='',
    license='',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The stretch_core package',
    entry_points={
        'console_scripts': [
            'stretch_driver = stretch_core.stretch_driver:main',
        ],
    },
)
