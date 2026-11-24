
import numpy as np
import time


# ============================================================
# 1. CALIBRATED PARAMETERS
# ============================================================

FORWARD_SPEED = 150
ROTATION_SPEED = 70

K_STOP_LEFT = 0.00400
K_STOP_RIGHT = 0.00450
BRAKING_TIME = 0.5

Kp_angle = 80
Kp_lateral = 300

MAX_CORRECTION = 30
MOTOR_BIAS = 0

DT = 0.05

# Provisional tolerances
WAYPOINT_TOLERANCE = 0.01
ANGLE_TOLERANCE = 0.02
MAX_FORWARD_ERROR = 0.35


# ============================================================
# 2. AUXILIARY FUNCTIONS
# ============================================================

def motors(left_speed=0, right_speed=0):
    return {
        "motor.left.target": [int(left_speed)],
        "motor.right.target": [int(right_speed)]
    }


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def initialize_path_state():
    return {
        "rotation_direction": None,
        "stop_error": 0.0,
        "brake_start": 0.0
    }


# ============================================================
# 3. MOVE ALONG PATH
# ============================================================

async def move_along_path(
    path,
    current_pos,
    current_theta,
    waypoint_index,
    mode,
    state,
    node,
    client,
    speed=FORWARD_SPEED
):

    """
    Follow an A* path using the current estimated robot pose.

    Parameters
    ----------
    path : array-like
        Waypoint coordinates in meters.

    current_pos : array-like
        Current robot position [x, y] in meters.

    current_theta : float
        Current robot orientation in radians.

    waypoint_index : int
        Index of the current target waypoint.

    mode : str
        Current navigation mode.

    state : dict
        Persistent navigation state.

    node :
        Connected and locked Thymio node.

    client :
        Active Thymio ClientAsync instance.

    speed : float
        Forward speed in Thymio motor units.

    Returns
    -------
    waypoint_index : int
        Updated waypoint index.

    mode : str
        Updated navigation mode.

    finished : bool
        True when the final waypoint is reached.
    """

    path = np.asarray(path, dtype=float)
    current_pos = np.asarray(current_pos, dtype=float)

    # --------------------------------------------------------
    # Input validation
    # --------------------------------------------------------

    if (
        path.ndim != 2
        or path.shape[1] != 2
        or len(path) < 2
        or not np.all(np.isfinite(path))
    ):
        await node.set_variables(motors(0, 0))
        raise ValueError("Invalid path")

    if (
        current_pos.shape != (2,)
        or not np.all(np.isfinite(current_pos))
        or not np.isfinite(current_theta)
    ):
        await node.set_variables(motors(0, 0))
        raise ValueError("Invalid robot pose")

    if mode == "STOP":
        await node.set_variables(motors(0, 0))
        return waypoint_index, mode, True

    if (
        not 1 <= waypoint_index < len(path)
        or mode not in (
            "ROTATE",
            "FORWARD",
            "BRAKE",
            "ROTATION_BRAKE"
        )
        or not 0 < speed <= 500
    ):
        await node.set_variables(motors(0, 0))
        raise ValueError("Invalid navigation state")

    # --------------------------------------------------------
    # Braking phase
    # --------------------------------------------------------

    if mode in ("BRAKE", "ROTATION_BRAKE"):

        await node.set_variables(motors(0, 0))

        elapsed = time.monotonic() - state["brake_start"]

        if elapsed >= BRAKING_TIME:

            if mode == "BRAKE":
                mode = "ROTATE"
            else:
                mode = "FORWARD"

            state["rotation_direction"] = None

        await client.sleep(DT)

        return waypoint_index, mode, False

    # --------------------------------------------------------
    # Waypoint management
    # --------------------------------------------------------

    while waypoint_index < len(path) and mode == "FORWARD":

        start = path[waypoint_index - 1]
        target = path[waypoint_index]

        segment = target - start
        length = np.linalg.norm(segment)

        if length < 1e-9:
            await node.set_variables(motors(0, 0))
            raise ValueError("Invalid path segment")

        direction = segment / length

        progress = np.dot(
            current_pos - start,
            direction
        )

        distance_to_target = np.linalg.norm(
            target - current_pos
        )

        # Final waypoint
        if waypoint_index == len(path) - 1:

            if distance_to_target <= WAYPOINT_TOLERANCE:

                await node.set_variables(motors(0, 0))

                return len(path), "STOP", True

            if progress >= length:

                await node.set_variables(motors(0, 0))

                raise RuntimeError(
                    "Final waypoint exceeded outside tolerance"
                )

            break

        # Direction of the next segment
        next_segment = path[waypoint_index + 1] - target

        if np.linalg.norm(next_segment) < 1e-9:

            await node.set_variables(motors(0, 0))

            raise ValueError("Invalid next segment")

        current_direction = np.arctan2(
            segment[1],
            segment[0]
        )

        next_direction = np.arctan2(
            next_segment[1],
            next_segment[0]
        )

        direction_change = normalize_angle(
            next_direction - current_direction
        )

        same_direction = abs(direction_change) < 1e-6

        # Continue without stopping on aligned waypoints
        if same_direction:

            if (
                distance_to_target <= WAYPOINT_TOLERANCE
                or progress >= length
            ):

                waypoint_index += 1

                continue

        # Stop before changing direction
        else:

            if distance_to_target <= WAYPOINT_TOLERANCE:

                waypoint_index += 1

                await node.set_variables(motors(0, 0))

                mode = "BRAKE"

                state["brake_start"] = time.monotonic()
                state["rotation_direction"] = None

                await client.sleep(DT)

                return waypoint_index, mode, False

            if progress >= length:

                await node.set_variables(motors(0, 0))

                raise RuntimeError(
                    "Turning waypoint exceeded outside tolerance"
                )

        break

    # --------------------------------------------------------
    # Angular and lateral errors
    # --------------------------------------------------------

    start = path[waypoint_index - 1]
    target = path[waypoint_index]

    dx = target[0] - start[0]
    dy = target[1] - start[1]

    segment_length = np.hypot(dx, dy)

    if segment_length < 1e-9:

        await node.set_variables(motors(0, 0))

        raise ValueError("Invalid segment")

    target_theta = np.arctan2(dy, dx)

    angle_error = normalize_angle(
        target_theta - current_theta
    )

    lateral_error = (
        dx * (current_pos[1] - start[1])
        - dy * (current_pos[0] - start[0])
    ) / segment_length

    # --------------------------------------------------------
    # Rotation on the spot
    # --------------------------------------------------------

    if mode == "ROTATE":

        if state["rotation_direction"] is None:

            if abs(angle_error) <= ANGLE_TOLERANCE:

                mode = "FORWARD"

            else:

                rotation_direction = (
                    1 if angle_error > 0 else -1
                )

                state["rotation_direction"] = (
                    rotation_direction
                )

                if rotation_direction == 1:

                    k_stop = K_STOP_LEFT

                else:

                    k_stop = K_STOP_RIGHT

                compensation = k_stop * ROTATION_SPEED

                state["stop_error"] = min(
                    compensation,
                    abs(angle_error) / 2
                )

        if mode == "ROTATE":

            rotation_direction = state[
                "rotation_direction"
            ]

            remaining_error = (
                rotation_direction * angle_error
            )

            # Anticipate rotation braking
            if remaining_error <= state["stop_error"]:

                await node.set_variables(motors(0, 0))

                mode = "ROTATION_BRAKE"

                state["brake_start"] = time.monotonic()

                await client.sleep(DT)

                return waypoint_index, mode, False

            turn = rotation_direction * ROTATION_SPEED

            left_speed = -turn
            right_speed = turn

    # --------------------------------------------------------
    # Angular safety during forward movement
    # --------------------------------------------------------

    elif abs(angle_error) > MAX_FORWARD_ERROR:

        await node.set_variables(motors(0, 0))

        mode = "BRAKE"

        state["brake_start"] = time.monotonic()
        state["rotation_direction"] = None

        await client.sleep(DT)

        return waypoint_index, mode, False

    # --------------------------------------------------------
    # Forward proportional control
    # --------------------------------------------------------

    if mode == "FORWARD":

        angular_correction = Kp_angle * angle_error

        lateral_correction = -Kp_lateral * lateral_error

        correction = (
            angular_correction
            + lateral_correction
        )

        correction = np.clip(
            correction,
            -MAX_CORRECTION,
            MAX_CORRECTION
        )

        correction = np.clip(
            correction + MOTOR_BIAS,
            -(speed - 1),
            speed - 1
        )

        left_speed = speed - correction
        right_speed = speed + correction

    # --------------------------------------------------------
    # Send motor commands
    # --------------------------------------------------------

    left_speed = int(
        np.clip(left_speed, -500, 500)
    )

    right_speed = int(
        np.clip(right_speed, -500, 500)
    )

    await node.set_variables(
        motors(left_speed, right_speed)
    )

    await client.sleep(DT)

    return waypoint_index, mode, False


