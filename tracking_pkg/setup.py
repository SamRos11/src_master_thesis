from setuptools import setup
import os
from glob import glob

package_name = 'tracking_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samu',
    maintainer_email='samu@todo.todo',
    description='Tracking package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_node = tracking_pkg.tracking_node:main'
        ],
    },
)

