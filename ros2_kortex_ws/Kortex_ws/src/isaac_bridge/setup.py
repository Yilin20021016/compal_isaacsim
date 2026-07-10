import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'isaac_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yilin',
    maintainer_email='werock2517@gmail.com',
    description='Kinova Gen3 Isaac Sim & MoveIt 2 bridge middleware',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'moveit_bridge = isaac_bridge.moveit_to_isaac_bridge:main',
        ],
    },
)
