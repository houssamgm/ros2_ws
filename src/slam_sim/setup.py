from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'slam_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nakoyonioka',
    maintainer_email='nakoyonioka@todo.todo',
    description='slam-toolbox package for simulating SLAM on 4Wheel Differential Drive Robot',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)