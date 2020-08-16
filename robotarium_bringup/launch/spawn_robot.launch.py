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

    pos_y = '1.45'
    neg_y = '-1.45'
    def_z = '0.91'
    x_val = [
        0.38 * -1, 0.38 * -2, 0.38 * -3, 0.38 * -4, 0.38 * -5,
        0.38 * 0.0,
        0.38 * 1, 0.38 * 2, 0.38 * 3, 0.38 * 4, 0.38 * 5]

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    worlds_file_path = os.path.join(
        get_package_share_directory('robotarium_gazebo'), 'worlds',
        'robotarium/robotarium.world')
    rclpy.logging.get_logger('Launch File').info(worlds_file_path)

    sdf_file_path = os.path.join(
        get_package_share_directory('robotarium_gazebo'), 'models',
        'gritsbotx/model.sdf')
    rclpy.logging.get_logger('Launch File').info(sdf_file_path)

    experiment_launch_description = LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', worlds_file_path, '-s',
                 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])

    for i in range(11):
        experiment_launch_description.add_entity(
            Node(
                package='robotarium_bringup',
                executable='spawn_gritsbotx',
                name='test_spawn',
                output='screen',
                namespace='',
                arguments=[
                    f'robot_{i}',  # Name
                    f'gb_{i}',  # Namespace
                    str(x_val[i]),  # X
                    pos_y,  # Y
                    def_z,  # Z
                    '0',  # Roll
                    '0',  # Pitch
                    '-1.57',  # Yaw
                ]
            )
        )

    for i in range(11):
        experiment_launch_description.add_entity(
            Node(
                package='robotarium_bringup',
                executable='spawn_gritsbotx',
                name='test_spawn',
                output='screen',
                namespace='',
                arguments=[
                    f'robot_{i + 11}',  # Name
                    f'gb_{i + 11}',  # Namespace
                    str(x_val[i]),  # X
                    neg_y,  # Y
                    def_z,  # Z
                    '0',  # Roll
                    '0',  # Pitch
                    '1.57',  # Yaw
                ]
            )
        )

    return experiment_launch_description
