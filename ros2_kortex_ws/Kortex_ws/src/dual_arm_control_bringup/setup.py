import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dual_arm_control_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.urdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.srdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.dae'))),
        (os.path.join('share', package_name, 'config', 'robotiq_description', 'visual', '2f_85'), glob(os.path.join('config', 'robotiq_description', 'visual', '2f_85', '*.dae'))),
        (os.path.join('share', package_name, 'config', 'robotiq_description', 'collision', '2f_85'), glob(os.path.join('config', 'robotiq_description', 'collision', '2f_85', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omniverse2',
    maintainer_email='werock2517@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
