from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diffbot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['models']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nakoyonioka',
    maintainer_email='nakoyonioka@todo.todo',
    description='4Wheel DiffDrive Robot Gazebo Simulation',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'guide_node = diffbot_sim.guide_node:main',
        ],
    },
)