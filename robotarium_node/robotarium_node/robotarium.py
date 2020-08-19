"""The Robotarium class used to instantiate experimentation.

Written by: The Robotarium Team
Modified by: Zahi Kakish (zmk5)

"""
import functools
import math
import time
from typing import Tuple

import numpy as np

import rclpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from robotarium_node.robotarium_abc import RobotariumABC

from robotarium_node.utilities.coordinates import quaternion_to_yaw
from robotarium_node.utilities.coordinates import yaw_to_quaternion


class Robotarium(RobotariumABC):
    """Provides runtime routines to interface with the Robotarium.

    NOTE: THIS CLASS SHOULD NEVER BE MODIFIED OR SUBMITTED!
    """

    def __init__(
            self,
            number_of_robots: int = -1,
            show_figure: bool = True,
            sim_in_real_time: bool = True,
            initial_conditions: np.ndarray = np.array([])) -> None:
        """Instantiate the Robotarium class."""
        super().__init__(
            number_of_robots, show_figure, sim_in_real_time, initial_conditions)

        # Initialize some rendering variables
        self.previous_render_time = time.time()
        self.sim_in_real_time = sim_in_real_time

        # Initialize checks for step and get poses calls
        self._called_step_already = True
        self._checked_poses_already = False

        # Initialization of error collection.
        self._errors = {}

        # Initialize steps
        self._iterations = 0

        # ROS2 Nodes, Publishers, and Subscribers
        rclpy.init()
        self._node = rclpy.create_node('robotarium_cl')

        self._pub = {}
        self._sub = {}
        self._msg = {}

        for i in range(number_of_robots):
            self._pub[i] = self._node.create_publisher(
                Twist, f'/gb_{i}/cmd_vel', 10)
            self._msg[i] = Twist()
            self._sub[i] = self._node.create_subscription(
                Odometry, f'/gb_{i}/odom',
                functools.partial(self._odom_callback, robot_id=i), 10)

    def get_poses(self) -> np.ndarray:
        """Return the states of the agents.

        Returns
        -------
        A `3xN` numpy array (of robot poses).
        """
        assert(not self._checked_poses_already), 'Can only call get_poses() once per call of step().'
        # Allow step() to be called again.
        self._called_step_already = False
        self._checked_poses_already = True

        return self.poses

    def call_at_scripts_end(self) -> None:
        """Call this function at the end of scripts to display potentail errors.

        Even if you don't want to print the errors, calling this function at
        the end of your script will enable execution on the Robotarium testbed.
        """
        self._node.get_logger().warn('##### DEBUG OUTPUT #####')
        self._node.get_logger().info(
            f'{math.ceil(self._iterations * 0.033)} real seconds when ' +
            'deployed on the Robotarium.')

        # Shutdown node and ROS2
        self._node.destroy_node()
        rclpy.shutdown()

        # print('##### DEBUG OUTPUT #####')
        # print('Your simulation will take approximately ' +
        #       f'{math.ceil(self._iterations*0.033)} real seconds when ' +
        #       'deployed on the Robotarium. \n')

        # if bool(self._errors):
        #     if 'boundary' in self._errors:
        #         print(f'\t Simulation had {self._errors["boundary"]} ' +
        #               f'{self._errors["boundary_string"]}\n')
        #     if 'collision' in self._errors:
        #         print(f'\t Simulation had {self._errors["collision"]} ' +
        #               f'{self._errors["collision_string"]}\n')
        #     if 'actuator' in self._errors:
        #         print(f'\t Simulation had {self._errors["actuator"]} ' +
        #               f'{self._errors["actuator_string"]}')
        # else:
        #     print('No errors in your simulation! '+ 
        #           'Acceptance of your experiment is likely!')

    def step(self) -> None:
        """Increment the simulation by updating the dynamics."""
        assert(not self._called_step_already), 'Make sure to call get_poses before calling step() again.'

        # Allow get_poses function to be called again.
        self._called_step_already = True
        self._checked_poses_already = False

        # Validate before thresholding velocities
        self._errors = self._validate()
        self._iterations += 1

        # Update dynamics of agents
        self.poses[0, :] = self.poses[0, :] + self.time_step * np.cos(self.poses[2, :]) * self.velocities[0, :]
        self.poses[1, :] = self.poses[1, :] + self.time_step * np.sin(self.poses[2, :]) * self.velocities[0, :]
        self.poses[2, :] = self.poses[2, :] + self.time_step * self.velocities[1, :]

        # Ensure angles are wrapped
        self.poses[2, :] = np.arctan2(
            np.sin(self.poses[2, :]), np.cos(self.poses[2, :]))

        # Update graphics
        if self.show_figure:  # TODO: Remove this.
            if self.sim_in_real_time:
                t = time.time()
                while t - self.previous_render_time < self.time_step:
                    t = time.time()
                self.previous_render_time = t

        self._publish_cmd_vel()
        rclpy.spin_once(self._node)

    def _publish_cmd_vel(self) -> None:
        """Publish the new poses of the robots to the Gazebo simulator."""
        for i in range(self.number_of_robots):
            self._msg[i].linear.x = self.velocities[0, i]
            self._msg[i].angular.z = self.velocities[1, i]
            self._pub[i].publish(self._msg[i])

    def _odom_callback(self, msg: Odometry, robot_id: int) -> None:
        """Set the pose of the robot from msg."""
        self.poses[0, robot_id] = msg.pose.pose.position.x
        self.poses[1, robot_id] = msg.pose.pose.position.y
        self.poses[2, robot_id] = quaternion_to_yaw(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )
