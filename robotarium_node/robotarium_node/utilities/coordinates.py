"""A series of conversion tools for Quaternions and Euler Angles.

These functions are modified versions of the algorithms found here:
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

Written by: Zahi Kakish (zmk5)

"""
from typing import Tuple

import numpy as np


def quaternion_to_roll(x: float, y: float, z: float, w: float) -> float:
    """Convert Quaternion to Roll Euler angle."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    return np.arctan2(sinr_cosp, cosr_cosp)


def quaternion_to_pitch(x: float, y: float, z: float, w: float) -> float:
    """Convert Quaternion to Pitch Euler angle."""
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1.0:
        return np.copysign(np.pi / 2, sinp)

    return np.arcsin(sinp)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert Quaternion to Yaw Euler angle."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def quaternion_to_euler(
        x: float,
        y: float,
        z: float,
        w: float) -> Tuple[float, float, float]:
    """Convert Quaternion to Euler angles."""
    return (
        quaternion_to_roll(x, y, z, w),
        quaternion_to_pitch(x, y, z, w),
        quaternion_to_yaw(x, y, z, w)
    )


def roll_to_quaternion(roll: float) -> Tuple[float, float, float, float]:
    """Convert only a Roll Euler angle to its respective Quaternion values."""
    c_r = np.cos(roll * 0.5)
    s_r = np.sin(roll * 0.5)
    c_p = 1.0
    s_p = 0.0
    c_y = 1.0
    s_y = 0.0

    return (
        s_r * c_p * c_y - c_r * s_p * s_y,  # x
        c_r * s_p * c_y + s_r * c_p * s_y,  # y
        c_r * c_p * s_y - s_r * s_p * c_y,  # z
        c_r * c_p * c_y + s_r * s_p * s_y,  # w
    )


def pitch_to_quaternion(pitch: float) -> Tuple[float, float, float, float]:
    """Convert only a Pitch Euler angle to its respective Quaternion values."""
    c_r = 1.0
    s_r = 0.0
    c_p = np.cos(pitch * 0.5)
    s_p = np.sin(pitch * 0.5)
    c_y = 1.0
    s_y = 0.0

    return (
        s_r * c_p * c_y - c_r * s_p * s_y,  # x
        c_r * s_p * c_y + s_r * c_p * s_y,  # y
        c_r * c_p * s_y - s_r * s_p * c_y,  # z
        c_r * c_p * c_y + s_r * s_p * s_y,  # w
    )


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert only a Yaw Euler angle to its respective Quaternion values."""
    c_r = 1.0
    s_r = 0.0
    c_p = 1.0
    s_p = 0.0
    c_y = np.cos(yaw * 0.5)
    s_y = np.sin(yaw * 0.5)

    return (
        s_r * c_p * c_y - c_r * s_p * s_y,  # x
        c_r * s_p * c_y + s_r * c_p * s_y,  # y
        c_r * c_p * s_y - s_r * s_p * c_y,  # z
        c_r * c_p * c_y + s_r * s_p * s_y,  # w
    )


def euler_to_quaternion(
        roll: float,
        pitch: float,
        yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles to Quaternion."""
    c_r = np.cos(roll * 0.5)
    s_r = np.sin(roll * 0.5)
    c_p = np.cos(pitch * 0.5)
    s_p = np.sin(pitch * 0.5)
    c_y = np.cos(yaw * 0.5)
    s_y = np.sin(yaw * 0.5)

    return (
        s_r * c_p * c_y - c_r * s_p * s_y,  # x
        c_r * s_p * c_y + s_r * c_p * s_y,  # y
        c_r * c_p * s_y - s_r * s_p * c_y,  # z
        c_r * c_p * c_y + s_r * s_p * s_y,  # w
    )
