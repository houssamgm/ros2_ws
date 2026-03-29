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
            'orgwoa_lidar = follow_target_nav2.orgwoa_lidar:main',
            'pid_dwb_tf = follow_target_nav2.pid_dwb_tf:main',
            'pid_dwb_lidar = follow_target_nav2.suivi_LIDAR_sans_mur_en_gazebo:main',
            'woa_dwb_tf = follow_target_nav2.woa_dwb_tf:main',
            'orgwoa_tf = follow_target_nav2.orgwoa_tf:main',
            'woa_dwb_lidar = follow_target_nav2.woa_dwb_lidar:main',
            'nav2_follow = follow_target_nav2.nav2_follow:main',
        ],
    },
)
