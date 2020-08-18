"""Unicycle with Boundary Example.

TODO: UPDATE DESCRIPTION

Written by: The Robotarium Team
Modified by: Zahi Kakish (zmk5)

"""
import numpy as np

from robotarium_node.robotarium import Robotarium
from robotarium_node.utilities.transformations import *
from robotarium_node.utilities.barrier_certificates import *
from robotarium_node.utilities.misc import *
from robotarium_node.utilities.controllers import *


def main() -> None:
    """Run script."""
    # Instantiate Robotarium object
    N = 5

    r = Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=True)

    # The robots will never reach their goal points so set iteration number
    iterations = 3000

    # Define goal points outside of the arena
    goal_points = np.array(np.mat('5 5 5 5 5; 5 5 5 5 5; 0 0 0 0 0'))

    # Create unicycle position controller
    unicycle_position_controller = create_clf_unicycle_position_controller()

    # Create barrier certificates to avoid collision
    uni_barrier_cert = create_unicycle_barrier_certificate_with_boundary()

    # define x initially
    x = r.get_poses()
    r.step()

    # While the number of robots at the required poses is less
    # than N...
    for _ in range(iterations):

        # Get poses of agents
        x = r.get_poses()

        # Create single-integrator control inputs
        dxu = unicycle_position_controller(x, goal_points[:2][:])

        # Create safe control inputs (i.e., no collisions)
        dxu = uni_barrier_cert(dxu, x)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu)

        # Iterate the simulation
        r.step()

    # Call at end of script to print debug information and for your script to run
    # on the Robotarium server properly.
    r.call_at_scripts_end()


if __name__ == '__main__':
    main()
