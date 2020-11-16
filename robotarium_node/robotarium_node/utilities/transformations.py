"""Functions to transform single-integrator to unicycle mappings.

Written by: The Robotarium Team
Modified by: Zahi Kakish (zmk5)

"""
from typing import Callable
from typing import Tuple
from typing import Union

import numpy as np


def create_si_to_uni_dynamics(
        linear_velocity_gain: Union[int, float] = 1,
        angular_velocity_limit: Union[int, float] = np.pi) -> Callable:
    """ Return a function mapping from single-integrator to unicycle dynamics.

    NOTE: Function mapping includes angular velocity magnitude restrictions.

    Parameters
    ----------
    linear_velocity_gain : int, float
        Gain for unicycle linear velocity
    angular_velocity_limit : int, float
        Limit for angular velocity (i.e., |w| < angular_velocity_limit)

    Returns
    -------
    A Callable function.

    """
    # Check user input types
    assert isinstance(linear_velocity_gain, (int, float)), f'In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be an integer or float. Recieved type {type(linear_velocity_gain).__name__}.'
    assert isinstance(angular_velocity_limit, (int, float)), f'In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must be an integer or float. Recieved type {type(angular_velocity_limit).__name__}.'

    # Check user input ranges/sizes
    assert linear_velocity_gain > 0, f'In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be positive. Recieved {linear_velocity_gain}.'
    assert angular_velocity_limit >= 0, f'In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must not be negative. Recieved {angular_velocity_limit}.'
    
    def si_to_uni_dyn(dxi: np.ndarray, poses: np.ndarray) -> np.ndarray:
        """Create mapping from single-integrator to unicycle dynamics.

        Parameters
        ----------
        dxi : np.ndarray
            2xN numpy array with single-integrator control inputs.
        poses : np.ndarray
            2xN numpy array with single-integrator poses.

        Returns
        -------
        2xN numpy array of unicycle control inputs

        """
        # Check user input types
        assert isinstance(dxi, np.ndarray), f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type {type(dxi).__name__}.'
        assert isinstance(poses, np.ndarray), f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the current robot poses (poses) must be a numpy array. Recieved type {type(poses).__name__}.'

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension {dxi.shape[0]}.'
        assert poses.shape[0] == 3, f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension {poses.shape[0]}.'
        assert dxi.shape[1] == poses.shape[1], f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size {dxi.shape[0]} x {dxi.shape[1]} and current pose array of size {poses.shape[0]} x {poses.shape[1]}.'

        M, N = np.shape(dxi)

        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = linear_velocity_gain*(a * dxi[0, :] + b * dxi[1, :])
        dxu[1, :] = angular_velocity_limit * np.arctan2(-b * dxi[0, :] + a * dxi[1, :], dxu[0, :]) / (np.pi / 2)

        return dxu

    return si_to_uni_dyn


def create_si_to_uni_dynamics_with_backwards_motion(
        linear_velocity_gain: Union[int, float] = 1,
        angular_velocity_limit: Union[int, float] = np.pi) -> Callable:
    """Returns a function mapping from single-integrator dynamics to unicycle dynamics.
    This implementation of the mapping allows for robots to drive backwards if
    that direction of linear velocity requires less rotation.

    Parameters
    ----------
    linear_velocity_gain : int
        Gain for unicycle linear velocity.
    angular_velocity_limit : float
        Limit for angular velocity (i.e., |w| < angular_velocity_limit).

    """
    #Check user input types
    assert isinstance(linear_velocity_gain, (int, float)), f'In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be an integer or float. Recieved type {type(linear_velocity_gain).__name__}.'
    assert isinstance(angular_velocity_limit, (int, float)), f'In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must be an integer or float. Recieved type {type(angular_velocity_limit).__name__}.'

    #Check user input ranges/sizes
    assert linear_velocity_gain > 0, f'In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be positive. Recieved {linear_velocity_gain}.'
    assert angular_velocity_limit >= 0, f'In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must not be negative. Recieved {angular_velocity_limit}.'

    def si_to_uni_dyn(dxi: np.ndarray, poses: np.ndarray) -> np.ndarray:
        """Create a mapping from single-integrator to unicycle dynamics.

        NOTE: Callable from `create_si_to_uni_dynamics_with_backwards_motion`.

        Parameters
        ----------
        dxi : np.ndarray
            2xN numpy array with single-integrator control inputs.
        poses : np.ndarray
            2xN numpy array with single-integrator poses.

        Returns
        -------
        2xN numpy array of unicycle control inputs.

        """
        #Check user input types
        assert isinstance(dxi, np.ndarray), f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type {type(dxi).__name__}.'
        assert isinstance(poses, np.ndarray), f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the current robot poses (poses) must be a numpy array. Recieved type {type(poses).__name__}.'

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension {dxi.shape[0]}.'
        assert poses.shape[0] == 3, f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension {poses.shape[0]}.'
        assert dxi.shape[1] == poses.shape[1], f'In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size {dxi.shape[0]} x {dxi.shape[1]} and current pose array of size {poses.shape[0]} x {poses.shape[1]}.'

        M, N = np.shape(dxi)

        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = linear_velocity_gain * (a * dxi[0, :] + b * dxi[1, :])
        dxu[1, :] = angular_velocity_limit * np.arctan2(-b * dxi[0, :] + a * dxi[1, :], dxu[0, :]) / (np.pi / 2)

        return dxu

    return si_to_uni_dyn


def create_si_to_uni_mapping(
        projection_distance: Union[int, float] = 0.05,
        angular_velocity_limit: Union[int, float] = np.pi) -> Tuple[Callable, Callable]:
    """Creates two functions for mapping from single integrator dynamics to
    unicycle dynamics and unicycle states to single integrator states.

    This mapping is done by placing a virtual control "point" in front of 
    the unicycle.

    Parameters
    ----------
    projection_distance : float
        How far ahead to place the point.
    angular_velocity_limit : float
        The maximum angular velocity that can be provided.

    Returns
    A Tuple of two Callable functions i.e. (function, function).

    """
    #Check user input types
    assert isinstance(projection_distance, (int, float)), 'In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r.' % type(projection_distance).__name__
    assert isinstance(angular_velocity_limit, (int, float)), 'In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be an integer or float. Recieved type %r.' % type(angular_velocity_limit).__name__
    
    #Check user input ranges/sizes
    assert projection_distance > 0, 'In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be positive. Recieved %r.' % projection_distance
    assert projection_distance >= 0, 'In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be greater than or equal to zero. Recieved %r.' % angular_velocity_limit

    def si_to_uni_dyn(dxi: np.ndarray, poses: np.ndarray) -> np.ndarray:
        """Transform single-integrator velocities to unicycle control inputs.

        Parameters
        ----------
        dxi : np.ndarray
            2xN numpy array of single-integrator control inputs
        poses : np.ndarray
            3xN numpy array of unicycle poses

        Returns
        -------
        2xN numpy array of unicycle control inputs.

        """
        #Check user input types
        assert isinstance(dxi, np.ndarray), f'In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type {type(dxi).__name__}.'
        assert isinstance(poses, np.ndarray), f'In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the current robot poses (poses) must be a numpy array. Recieved type {type(poses).__name__}.'

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, f'In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension {dxi.shape[0]}.'
        assert poses.shape[0] == 3, f'In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension {poses.shape[0]}.'
        assert dxi.shape[1] == poses.shape[1], f'In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size {dxi.shape[0]} x {dxi.shape[1]} and current pose array of size {poses.shape[0]} x {poses.shape[1]}.'

        M, N = np.shape(dxi)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = (cs * dxi[0, :] + ss * dxi[1, :])
        dxu[1, :] = (1 / projection_distance) * (-ss * dxi[0, :] + cs * dxi[1, :])

        #Impose angular velocity cap.
        dxu[1, dxu[1, :] > angular_velocity_limit] = angular_velocity_limit
        dxu[1, dxu[1, :] < -angular_velocity_limit] = -angular_velocity_limit

        return dxu

    def uni_to_si_states(poses: np.ndarray) -> np.ndarray:
        """Transform unicycle states to single-integrator states.

        Parameters
        ----------
        poses : np.ndarray
            3xN numpy array of unicycle states

        Returns
        -------
        2xN numpy array of single-integrator states.

        """
        _, N = np.shape(poses)

        si_states = np.zeros((2, N))
        si_states[0, :] = poses[0, :] + projection_distance * np.cos(poses[2, :])
        si_states[1, :] = poses[1, :] + projection_distance * np.sin(poses[2, :])

        return si_states

    return si_to_uni_dyn, uni_to_si_states


def create_uni_to_si_dynamics(projection_distance: float = 0.05) -> Callable:
    """Creates two functions for mapping from unicycle dynamics to single 
    integrator dynamics and single integrator states to unicycle states. 

    This mapping is done by placing a virtual control "point" in front of 
    the unicycle.

    Parameters
    ----------
    projection_distance : float
        How far ahead to place the point.

    Returns
    -------
    A Callable function.

    """
    # Check user input types
    assert isinstance(projection_distance, (int, float)), f'In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type {type(projection_distance).__name__}.'

    # Check user input ranges/sizes
    assert projection_distance > 0, f'In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be positive. Recieved {projection_distance}.'

    def uni_to_si_dyn(dxu: np.ndarray, poses: np.ndarray) -> np.ndarray:
        """A function for converting from unicycle to single-integrator dynamics.
        Utilizes a virtual point placed in front of the unicycle.

        Parameters
        ----------
        dxu : np.ndarray
            2xN numpy array of unicycle control inputs.
        poses : np.ndarray
            3xN numpy array of unicycle poses.
        projection_distance : float
            How far ahead of the unicycle model to place the point.

        Returns
        -------
        2xN numpy array of single-integrator control inputs.

        """
        # Check user input types
        assert isinstance(dxu, np.ndarray), f'In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the unicycle velocity inputs (dxu) must be a numpy array. Recieved type {type(dxu).__name__}.'
        assert isinstance(poses, np.ndarray), f'In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the current robot poses (poses) must be a numpy array. Recieved type {type(poses).__name__}.'

        #Check user input ranges/sizes
        assert dxu.shape[0] == 2, f'In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the unicycle velocity inputs (dxu) must be 2 ([v;w]). Recieved dimension {dxu.shape[0]}.'
        assert poses.shape[0] == 3, f'In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension {poses.shape[0]}.'
        assert dxu.shape[1] == poses.shape[1], f'In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the number of unicycle velocity inputs must be equal to the number of current robot poses. Recieved a unicycle velocity input array of size {dxu.shape[0]} x {dxu.shape[1]} and current pose array of size {poses.shape[0]} x {poses.shape[1]}.'

        M,N = np.shape(dxu)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxi = np.zeros((2, N))
        dxi[0, :] = (cs * dxu[0, :] - projection_distance * ss * dxu[1, :])
        dxi[1, :] = (ss * dxu[0, :] + projection_distance * cs * dxu[1, :])

        return dxi

    return uni_to_si_dyn
