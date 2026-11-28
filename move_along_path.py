
import numpy as np
import time


# ==========================================================
# 1. PARAMETERS
# ==========================================================

FORWARD_SPEED = 150
ROTATION_SPEED = 70

K_STOP_LEFT = 0.00400
K_STOP_RIGHT = 0.00450
BRAKING_TIME = 0.5

Kp_angle = 80
Kp_lateral = 300
MAX_CORRECTION = 30

DT = 0.05
WAYPOINT_TOLERANCE = 0.010
ANGLE_TOLERANCE = 0.020
MAX_FORWARD_ERROR = 0.35


# ==========================================================
# 2. AUXILIARY FUNCTIONS
# ==========================================================

def motors(left_speed=0, right_speed=0):
    return {
        "motor.left.target": [int(np.clip(left_speed, -500, 500))],
        "motor.right.target": [int(np.clip(right_speed, -500, 500))]
    }


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def initialize_path_state():
    return {
        "rotation_direction": None,
        "stop_error": 0.0,
        "brake_start": 0.0
    }


# ==========================================================
# 3. PATH FOLLOWING
# ==========================================================

async def move_along_path(
    path, current_pos, current_theta,
    waypoint_index, mode, state,
    node, client, speed=FORWARD_SPEED
):
    path = np.asarray(path, dtype=float)
    pos = np.asarray(current_pos, dtype=float)

    # Validate inputs.
    if (
        path.ndim != 2 or path.shape[1] != 2
        or len(path) < 2
        or not np.all(np.isfinite(path))
        or pos.shape != (2,)
        or not np.all(np.isfinite(pos))
        or current_theta is None
        or not np.isfinite(current_theta)
        or not 0 < speed <= 500
        or not 1 <= waypoint_index <= len(path)
        or mode not in (
            "FORWARD", "ROTATE", "BRAKE",
            "ROTATION_BRAKE", "STOP"
        )
    ):
        await node.set_variables(motors())
        raise ValueError("Invalid navigation inputs")

    # Stop the robot and start a braking phase.
    async def brake(next_mode):
        await node.set_variables(motors())
        state["brake_start"] = time.monotonic()
        state["rotation_direction"] = None
        await client.sleep(DT)
        return waypoint_index, next_mode, False

    # Final destination.
    if (
        mode == "STOP"
        or waypoint_index == len(path)
        or np.linalg.norm(path[-1] - pos) <= WAYPOINT_TOLERANCE
    ):
        await node.set_variables(motors())
        return len(path), "STOP", True

    # Wait until the robot has stopped.
    if mode in ("BRAKE", "ROTATION_BRAKE"):

        await node.set_variables(motors())

        if time.monotonic() - state["brake_start"] >= BRAKING_TIME:
            mode = "ROTATE"

        await client.sleep(DT)
        return waypoint_index, mode, False

    # Manage waypoints.
    while True:

        start = path[waypoint_index - 1]
        target = path[waypoint_index]

        segment = target - start
        length = np.linalg.norm(segment)

        if length < 1e-9:
            await node.set_variables(motors())
            raise ValueError("Invalid path segment")

        direction = segment / length
        distance = np.linalg.norm(target - pos)
        progress = np.dot(pos - start, direction)

        if mode != "FORWARD":
            break

        if waypoint_index == len(path) - 1:
            if progress >= length:
                await node.set_variables(motors())
                raise RuntimeError("Final waypoint exceeded")
            break

        next_segment = path[waypoint_index + 1] - target
        next_length = np.linalg.norm(next_segment)

        if next_length < 1e-9:
            await node.set_variables(motors())
            raise ValueError("Invalid next segment")

        same_direction = np.allclose(
            direction, next_segment / next_length,
            atol=1e-6
        )

        # Continue without stopping on straight segments.
        if same_direction and (
            distance <= WAYPOINT_TOLERANCE
            or progress >= length
        ):
            waypoint_index += 1
            continue

        # Stop before changing direction.
        if distance <= WAYPOINT_TOLERANCE:
            waypoint_index += 1
            return await brake("BRAKE")

        if progress >= length:
            await node.set_variables(motors())
            raise RuntimeError("Turning waypoint exceeded")

        break

    # Angular and lateral errors.
    target_theta = np.arctan2(direction[1], direction[0])

    angle_error = normalize_angle(
        target_theta - current_theta
    )

    lateral_error = (
        direction[0] * (pos[1] - start[1])
        - direction[1] * (pos[0] - start[0])
    )

    # Rotation on the spot.
    if mode == "ROTATE":

        if abs(angle_error) <= ANGLE_TOLERANCE:
            mode = "FORWARD"
            state["rotation_direction"] = None

        else:
            if state["rotation_direction"] is None:

                sign = np.sign(angle_error)

                k_stop = (
                    K_STOP_LEFT if sign > 0
                    else K_STOP_RIGHT
                )

                state["rotation_direction"] = sign
                state["stop_error"] = min(
                    k_stop * ROTATION_SPEED,
                    abs(angle_error) / 2
                )

            sign = state["rotation_direction"]

            if sign * angle_error <= state["stop_error"]:
                return await brake("ROTATION_BRAKE")

            # Reduce rotation speed for small remaining errors.
            turn = sign * min(
                ROTATION_SPEED,
                max(20, Kp_angle * abs(angle_error))
            )

            await node.set_variables(motors(-turn, turn))
            await client.sleep(DT)

            return waypoint_index, "ROTATE", False

    # Stop and realign if angular error is too large.
    if abs(angle_error) > MAX_FORWARD_ERROR:
        return await brake("BRAKE")

    # Continuous trajectory correction.
    correction = (
        Kp_angle * angle_error
        - Kp_lateral * lateral_error
    )

    limit = min(MAX_CORRECTION, max(0, speed - 1))
    correction = np.clip(correction, -limit, limit)

    await node.set_variables(motors(
        speed - correction,
        speed + correction
    ))

    await client.sleep(DT)

    return waypoint_index, "FORWARD", False
