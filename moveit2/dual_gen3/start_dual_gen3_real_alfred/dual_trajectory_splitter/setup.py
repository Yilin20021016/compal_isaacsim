from setuptools import setup

package_name = 'dual_trajectory_splitter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfred',
    maintainer_email='alfred@example.com',
    description='Trajectory splitter for dual Kinova Gen3 arms',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            "clean_trajectory_splitter = dual_trajectory_splitter.trajectory_splitter_node_clean:main",'trajectory_splitter_node = dual_trajectory_splitter.trajectory_splitter_node:main',
        ],
    },
)
