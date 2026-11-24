import cv2
import numpy as np
from collections import deque

def predict_position(x, y, theta, d_left, d_right,
                     wheel_base=95, cell_size=15):

    """
    x, y: Previous position in grid cells
    theta: Previous orientation in degrees
    d_left, d_right: Wheel displacements in mm
    wheel_base: Distance between wheels in mm
    cell_size: Size of a maze cell in mm

    Returns: x_new, y_new, theta_new
    """

    # Convert angle to radians
    theta = np.radians(theta)

    # Distance traveled by the robot center
    d = (d_left + d_right) / 2

    # Change in orientation
    d_theta = (d_right - d_left) / wheel_base

    # Orientation halfway through the movement
    theta_mid = theta + d_theta / 2

    # Convert displacement into grid cells
    d_cells = d / cell_size

    # Predict position
    x_new = x + d_cells * np.cos(theta_mid)
    y_new = y - d_cells * np.sin(theta_mid)

    # Predict orientation
    theta_new = np.degrees(theta + d_theta) % 360

    return x_new, y_new, theta_new

def predict_from_speeds(x, y, theta,
                        speed_left, speed_right, dt):

    # Convert Thymio speed units to mm/s
    speed_factor = 0.4

    v_left = speed_left * speed_factor
    v_right = speed_right * speed_factor

    # Wheel displacement in mm
    d_left = v_left * dt
    d_right = v_right * dt

    return predict_position(
        x, y, theta,
        d_left, d_right
    )