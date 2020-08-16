"""Script used to spawn any robot in a user-defined position.

Written by: Zahi Kakish (zmk5)
"""
import sys

from gazebo_msgs.srv import SpawnEntity

import rclpy


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

    # Get path to robot from launch file.
    sdf_file_path = argv[0]

    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[1]  # 'gritsbot_0'
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[2]  # 'gb_0'
    request.initial_pose.position.x = float(argv[3])  # 5.0
    request.initial_pose.position.y = float(argv[4])  # 5.0
    request.initial_pose.position.z = float(argv[5])  # 0.1

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
