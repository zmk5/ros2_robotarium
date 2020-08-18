"""Setup File for robotarium_examples."""
import os
from glob import glob

from setuptools import setup


PACKAGE_NAME = 'robotarium_examples'

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
            'barrier_certificates = ' + \
                'robotarium_examples.barrier_certificates.barrier_certificates:main',
            'si_barriers_with_boundary = ' + \
                'robotarium_examples.barrier_certificates.si_barriers_with_boundary_example:main',
            'uni_barriers_with_boundary = ' + \
                'robotarium_examples.barrier_certificates.uni_barriers_with_boundary_example:main',
            'uni_dd_barriers_with_boundary = ' + \
                'robotarium_examples.barrier_certificates.uni_dd_barrier_with_boundary:main',
            'consensus = ' + \
                'robotarium_examples.consensus.consensus:main',
            'consensus_fewer_errors = ' + \
                'robotarium_examples.consensus_fewer_errors.consensus_fewer_errors:main',
            'leader_follower_save_data = ' + \
                'robotarium_examples.data_saving.leader_follower_save_data:main',
            'formation_control = ' + \
                'robotarium_examples.formation_control.formation_control:main',
            'si_go_to_point = ' + \
                'robotarium_examples.go_to_point.si_go_to_point:main',
            'uni_go_to_point = ' + \
                'robotarium_examples.go_to_point.uni_go_to_point:main',
            'uni_go_to_pose_clf = ' + \
                'robotarium_examples.go_to_pose.uni_go_to_pose_clf:main',
            'uni_go_to_pose_hybrid = ' + \
                'robotarium_examples.go_to_pose.uni_go_to_pose_hybrid:main',
            'leader_follower = ' + \
                'robotarium_examples.leader_follower_static.leader_follower:main',
        ],
    },
)
