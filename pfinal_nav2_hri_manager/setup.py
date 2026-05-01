from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pfinal_nav2_hri_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.es',
    description='Proyecto final ROS2 Jazzy: Nav2 + HRI + YOLO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = pfinal_nav2_hri_manager.mission_manager_node:main',
            'hri_test         = pfinal_nav2_hri_manager.hri_test_node:main',
            'nav_test         = pfinal_nav2_hri_manager.nav_test_node:main',
        ],
    },
)
