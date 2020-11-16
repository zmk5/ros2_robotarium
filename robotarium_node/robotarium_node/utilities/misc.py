"""Various other functions to use.

Written by: The Robotarium Team
Modified by: Zahi Kakish (zmk5)

"""
from typing import Tuple
from typing import Union

import numpy as np


def generate_initial_conditions(
        N: int,
        spacing: Union[int, float] = 0.3,
        width: Union[int, float] = 3,
        height: Union[int, float] = 1.8) -> np.ndarray:
    """Generate random initial condition poses for a specified arena size.

    Parameters
    ----------
    N : int
        Number of agents.
    spacing : int, float
        How far apart initial positions can be.
    width: int, float
        Width of arena.
    height: int, float
        Height of area.

    Returns
    -------
    3xN numpy array (of poses)

    """
    # Check user input types
    assert isinstance(N, int), f'In the function generate_initial_conditions, the number of robots (N) to generate intial conditions for must be an integer. Recieved type {type(N).__name__}.'
    assert isinstance(spacing, (float, int)), f'In the function generate_initial_conditions, the minimum spacing between robots (spacing) must be an integer or float. Recieved type {type(spacing).__name__}.'
    assert isinstance(width, (float, int)), f'In the function generate_initial_conditions, the width of the area to place robots randomly (width) must be an integer or float. Recieved type {type(width).__name__}.'
    assert isinstance(height, (float, int)), f'In the function generate_initial_conditions, the height of the area to place robots randomly (width) must be an integer or float. Recieved type {type(height).__name__}.'

    # Check user input ranges/sizes
    assert N > 0, f'In the function generate_initial_conditions, the number of robots to generate initial conditions for (N) must be positive. Recieved {N}.'
    assert spacing > 0, f'In the function generate_initial_conditions, the spacing between robots (spacing) must be positive. Recieved {spacing}.'
    assert width > 0, f'In the function generate_initial_conditions, the width of the area to initialize robots randomly (width) must be positive. Recieved {width}.'
    assert height > 0, f'In the function generate_initial_conditions, the height of the area to initialize robots randomly (height) must be positive. Recieved {height}.'

    x_range = int(np.floor(width / spacing))
    y_range = int(np.floor(height / spacing))

    assert x_range != 0, 'In the function generate_initial_conditions, the space between robots (space) is too large compared to the width of the area robots are randomly initialized in (width).'
    assert y_range != 0, 'In the function generate_initial_conditions, the space between robots (space) is too large compared to the height of the area robots are randomly initialized in (height).'
    assert x_range*y_range > N, f'In the function generate_initial_conditions, it is impossible to place {N} robots within a {width} x {height} meter area with a spacing of {spacing} meters.'

    choices = (np.random.choice(x_range * y_range, N, replace=False) + 1)

    poses = np.zeros((3, N))

    for i, c in enumerate(choices):
        x, y = divmod(c, y_range)
        poses[0, i] = x * spacing - width / 2
        poses[1, i] = y * spacing - height / 2
        poses[2, i] = np.random.rand() * 2 * np.pi - np.pi

    return poses


def at_pose(
        states: np.ndarray,
        poses: np.ndarray,
        position_error: Union[int, float] = 0.05,
        rotation_error: Union[int, float] = 0.2) -> Tuple[np.ndarray]:
    """Check whether robots are "close enough" to poses.

    Parameters
    ----------
    states : np.ndarray
        3xN numpy array of unicycle states.
    poses : np.ndarray
        3xN numpy array of desired states.

    Returns
    -------
    A 1xN numpy index array of agents that are close enough.

    """
    # Check user input types
    assert isinstance(states, np.ndarray), f'In the at_pose function, the robot current state argument (states) must be a numpy ndarray. Recieved type {type(states).__name__}.'
    assert isinstance(poses, np.ndarray), f'In the at_pose function, the checked pose argument (poses) must be a numpy ndarray. Recieved type {type(poses).__name__}.'
    assert isinstance(position_error, (float, int)), f'In the at_pose function, the allowable position error argument (position_error) must be an integer or float. Recieved type {type(position_error).__name__}.'
    assert isinstance(rotation_error, (float, int)), f'In the at_pose function, the allowable angular error argument (rotation_error) must be an integer or float. Recieved type {type(rotation_error).__name__}.'

    # Check user input ranges/sizes
    assert states.shape[0] == 3, f'In the at_pose function, the dimension of the state of each robot must be 3 ([x;y;theta]). Recieved {states.shape[0]}.'
    assert poses.shape[0] == 3, f'In the at_pose function, the dimension of the checked pose of each robot must be 3 ([x;y;theta]). Recieved {poses.shape[0]}.'
    assert states.shape == poses.shape, f'In the at_pose function, the robot current state and checked pose inputs must be the same size (3xN, where N is the number of robots being checked). Recieved a state array of size {states.shape[0]} x {states.shape[1]} and checked pose array of size {poses.shape[0]} x {poses.shape[1]}.'

    # Calculate rotation errors with angle wrapping
    res = states[2, :] - poses[2, :]
    res = np.abs(np.arctan2(np.sin(res), np.cos(res)))

    # Calculate position errors
    pes = np.linalg.norm(states[:2, :] - poses[:2, :], 2, 0)

    # Determine which agents are done
    done = np.nonzero((res <= rotation_error) & (pes <= position_error))

    return done


def at_position(
        states: np.ndarray,
        points: np.ndarray,
        position_error: Union[int, float] = 0.02) -> np.ndarray:
    """Check whether robots are "close enough" to desired position.

    Parameters
    ----------
    states : np.ndarray
        3xN numpy array of unicycle states.
    points : np.ndarray
        2xN numpy array of desired points.

    Returns
    -------
    A 1xN numpy index array of agents that are close enough to final position.
    """
    # Check user input types
    assert isinstance(states, np.ndarray), f'In the at_position function, the robot current state argument (states) must be a numpy ndarray. Recieved type {type(states).__name__}.'
    assert isinstance(points, np.ndarray), f'In the at_position function, the desired pose argument (poses) must be a numpy ndarray. Recieved type {type(points).__name__}.'
    assert isinstance(position_error, (float, int)), f'In the at_position function, the allowable position error argument (position_error) must be an integer or float. Recieved type {type(position_error).__name__}.'
    
    # Check user input ranges/sizes
    assert states.shape[0] == 3, f'In the at_position function, the dimension of the state of each robot (states) must be 3. Recieved {states.shape[0]}.'
    assert points.shape[0] == 2, f'In the at_position function, the dimension of the checked position for each robot (points) must be 2. Recieved {points.shape[0]}.'
    assert states.shape[1] == points.shape[1], f'In the at_position function, the number of checked points (points) must match the number of robot states provided (states). Recieved a state array of size {states.shape[0]} x {states.shape[1]} and desired pose array of size {points.shape[0]} x {points.shape[1]}.'

    # Calculate position errors
    pes = np.linalg.norm(states[:2, :] - points, 2, 0)

    # Determine which agents are done
    done = np.nonzero((pes <= position_error))

    return done
