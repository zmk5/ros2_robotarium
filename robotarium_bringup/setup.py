"""Setup File for robotarium_bringup."""
import os
from glob import glob

from setuptools import setup


PACKAGE_NAME = 'robotarium_bringup'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME,
         ['package.xml']),
        # (os.path.join('share', PACKAGE_NAME, 'config'),
        #  glob('config/*.yaml')),
        (os.path.join('share', PACKAGE_NAME, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_gritsbotx = robotarium_bringup.spawn_gritsbotx:main',
            'spawn_robot = robotarium_bringup.spawn_robot:main',
        ],
    },
)
