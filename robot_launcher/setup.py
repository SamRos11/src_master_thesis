import os
from glob import glob
from setuptools import setup

package_name = 'robot_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Registra il pacchetto nell'indice di ROS
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installa il package.xml
        ('share/' + package_name, ['package.xml']),
        
        # --- PARTE CRUCIALE PER I LAUNCH FILE ---
        # Copia tutti i file .launch.py da 'launch/' a 'share/robot_launcher/launch/'
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # --- PARTE CRUCIALE PER I CONFIG ---
        # Copia tutti i file da 'config/' a 'share/robot_launcher/config/'
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuele',
    maintainer_email='samuele@todo.todo',
    description='Launcher package for robot sensors and slam',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)