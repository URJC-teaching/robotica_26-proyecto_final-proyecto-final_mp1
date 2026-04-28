from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pfinal_nav2_hri_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='alumno',
    maintainer_email='alumno@example.com',
    description='Coordinator package for HRI, Nav2 and YOLO person following.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_coordinator = pfinal_nav2_hri_manager.mission_coordinator:main',
        ],
    },
)
