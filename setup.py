import os
from glob import glob
from setuptools import setup

package_name = 'target_flight_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jekim98',
    maintainer_email='bak1916@gmail.com',
    description='Target aircraft flight simulator — publishes position/velocity along a configurable start-to-goal track',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_sim_node = target_flight_sim.target_sim_node:main',
        ],
    },
)
