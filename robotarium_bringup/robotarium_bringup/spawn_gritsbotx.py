"""Script used to spawn a GRITSBot X in a user-defined position.

Written by: Zahi Kakish (zmk5)
"""
import os
import sys
from typing import Tuple

from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.srv import SpawnEntity

import numpy as np

import rclpy


def to_quaternion(
        roll: float,
        pitch: float,
        yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler to Quaternion."""
    c_r = np.cos(roll * 0.5)
    s_r = np.sin(roll * 0.5)
    c_p = np.cos(pitch * 0.5)
    s_p = np.sin(pitch * 0.5)
    c_y = np.cos(yaw * 0.5)
    s_y = np.sin(yaw * 0.5)

    x = s_r * c_p * c_y - c_r * s_p * s_y
    y = c_r * s_p * c_y + s_r * c_p * s_y
    z = c_r * c_p * s_y - s_r * s_p * c_y
    w = c_r * c_p * c_y + s_r * s_p * s_y

    return (x, y, z, w)


def main():
    """Spawn robot onto gazebo field."""
    # Get input arguments from user
    # TODO: Try and figure out how to use argparse with this
    argv = sys.argv[1:]

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    # Get path to the GRITSBot X
    sdf_file_path = os.path.join(
        get_package_share_directory('robotarium_gazebo'),
        'models',
        'gritsbotx/model.sdf')

    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[0]  # 'gritsbot_0'
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]  # 'gb_0'

    # Set robot postion
    request.initial_pose.position.x = float(argv[2])  # 5.0
    request.initial_pose.position.y = float(argv[3])  # 5.0
    request.initial_pose.position.z = float(argv[4])  # 0.1

    # Set robot orientation
    quaternion = to_quaternion(float(argv[5]), float(argv[6]), float(argv[7]))
    request.initial_pose.orientation.x = quaternion[0]
    request.initial_pose.orientation.y = quaternion[1]
    request.initial_pose.orientation.z = quaternion[2]
    request.initial_pose.orientation.w = quaternion[3]

    node.get_logger().info('Sending service request to `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f'response: {future.result()}')

    else:
        raise RuntimeError(
            f'exception while calling service: {future.exception()}')

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
