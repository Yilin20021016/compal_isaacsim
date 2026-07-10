from setuptools import setup
from glob import glob
import os

package_name = 'dual_gen3_moveit_config'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfred',
    maintainer_email='alfred@example.com',
    description='Dual Gen3 MoveIt config',
    license='BSD',
)
