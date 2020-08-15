"""Launch file for a gritsbot in an empty world.

Written by: Zahi Kakish (zmk5)
"""
import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import rclpy


def get_args():
    """Get arguments for simulation deployment."""
    parser = argparse.ArgumentParser(
        description='Robot Gazebo Launch File.'
    )

    # Required arguments
    parser.add_argument('-n', '--number',
                        action='store',
                        type=str,
                        required=False,
                        help='Arena number: [0, 1, 2]',
                        default='0')

    parser.add_argument('-f', '--file',
                        action='store',
                        type=str,
                        required=False,
                        help='YAML Parameter file',
                        default='gains_sim.yaml')

    return parser.parse_args(sys.argv[4:])


def generate_launch_description():
    """Launch file for gritsbot in Gazebo."""
    # Get input arguments
    input_args = get_args()

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    worlds_file_path = os.path.join(
        get_package_share_directory('robotarium_gazebo'), 'worlds',
        'empty/empty.world')
    rclpy.logging.get_logger('Launch File').info(worlds_file_path)

    sdf_file_path = os.path.join(
        get_package_share_directory('robotarium_gazebo'), 'models',
        'gritsbotx/model.sdf')
    rclpy.logging.get_logger('Launch File').info(sdf_file_path)

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', worlds_file_path, '-s',
                 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time',
        #          use_sim_time],
        #     output='screen'
        # ),
        Node(
            package='herding_ros_gazebo',
            executable='spawn_robot',
            name='test_spawn',
            output='screen',
            namespace='',
            arguments=[
                'robot_0',  # Name
                '',  # Namespace
                '0.0',  # X
                '0.0',  # Y
                '0.0',  # Z
            ]
        ),
    ])
