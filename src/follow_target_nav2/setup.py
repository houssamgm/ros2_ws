from setuptools import find_packages, setup

package_name = 'follow_target_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shisui',
    maintainer_email='shisuigh04@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uwb_distance = follow_target_nav2.uwb_distance:main',
            'pid_follow = follow_target_nav2.pid_follow:main',
            'lidar_follow = follow_target_nav2.lidar_follow:main',
            'moving_goal = follow_target_nav2.moving_goal:main',
        ],
    },
)
