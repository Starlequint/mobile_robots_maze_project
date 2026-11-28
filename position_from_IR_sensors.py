
import numpy as np


# ==========================================================
# 1. PARAMETERS AND CALIBRATION
# ==========================================================

IR_THRESHOLD = 500
SENSOR_IDS = [1, 2, 3]

SENSOR_OFFSETS = {
    1: np.array([0.075,  0.027]),
    2: np.array([0.080,  0.000]),
    3: np.array([0.075, -0.027])
}

IR_ANGLES = {
    1: np.deg2rad(25),
    2: 0.0,
    3: np.deg2rad(-25)
}

DISTANCE_TOLERANCE = {1: 0.020, 2: 0.015, 3: 0.015}
IR_WEIGHTS = {1: 0.15, 2: 0.50, 3: 0.35}

SEARCH_RADIUS = 0.070
ANGLE_RANGE = np.deg2rad(25)
MAX_IR_DISTANCE = 0.160
MAX_MAP_DISTANCE = 0.30
MAP_TOLERANCE = 0.020

IR_CALIBRATION = {
    1: {
        "distances": np.array([
            0.02, 0.03, 0.04, 0.05,
            0.06, 0.08, 0.09, 0.10
        ]),
        "values": np.array([
            4819.6, 4588.3, 4005.7, 3334.8,
            2923.2, 2182.2, 1998.4, 1453.0
        ])
    },
    2: {
        "distances": np.array([
            0.02, 0.03, 0.04, 0.05,
            0.06, 0.08, 0.09, 0.10,
            0.11, 0.12, 0.13, 0.14,
            0.15, 0.16
        ]),
        "values": np.array([
            4870.9, 4761.1, 4462.8, 3910.0,
            3617.6, 3048.0, 2857.6, 2642.5,
            2479.8, 2295.4, 2120.2, 1900.0,
            1689.2, 1279.1
        ])
    },
    3: {
        "distances": np.array([
            0.02, 0.03, 0.04, 0.05,
            0.06, 0.08, 0.09, 0.10,
            0.11, 0.12
        ]),
        "values": np.array([
            4907.6, 4519.3, 4204.8, 3594.1,
            3368.8, 2748.0, 2391.8, 2350.2,
            1983.3, 1693.3
        ])
    }
}


# ==========================================================
# 2. AUXILIARY FUNCTIONS
# ==========================================================

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def direction(theta):
    return np.array([np.cos(theta), np.sin(theta)])


def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def frontal_to_axial_distance(i, distance):
    offset = SENSOR_OFFSETS[2][0] - SENSOR_OFFSETS[i][0]
    return (distance + offset) / np.cos(IR_ANGLES[i])


def ir_to_distance(i, value):
    if i not in IR_CALIBRATION:
        return None

    calibration = IR_CALIBRATION[i]
    values = calibration["values"]

    if not np.isfinite(value) or value < IR_THRESHOLD:
        return None

    if not values[-1] <= value <= values[0]:
        return None

    distances = frontal_to_axial_distance(
        i, calibration["distances"]
    )

    return float(np.interp(
        value, values[::-1], distances[::-1]
    ))


def valid_pose(pos, theta):
    return (
        pos is not None and theta is not None
        and np.asarray(pos).shape == (2,)
        and np.all(np.isfinite(pos))
        and np.isfinite(theta)
    )


def cell_is_free(pos, obstacle_map, resolution):
    col, row = np.floor(pos / resolution).astype(int)

    return (
        0 <= row < obstacle_map.shape[0]
        and 0 <= col < obstacle_map.shape[1]
        and obstacle_map[row, col] == 0
    )


def distance_to_wall(
    pos, theta, obstacle_map,
    resolution, max_dist=MAX_MAP_DISTANCE
):
    ray = direction(theta)
    step = min(resolution / 2, 0.001)

    for d in np.arange(0, max_dist + step, step):
        d = min(d, max_dist)

        col, row = np.floor(
            (pos + d * ray) / resolution
        ).astype(int)

        if not (
            0 <= row < obstacle_map.shape[0]
            and 0 <= col < obstacle_map.shape[1]
        ):
            break

        if obstacle_map[row, col] == 1:
            return float(d), (row, col)

    return None, None


def identify_wall(origin, theta, cell, resolution):
    if cell is None:
        return None

    row, col = cell
    ray = direction(theta)

    lower = np.array([col, row]) * resolution
    boundary = np.where(ray > 0, lower, lower + resolution)

    entries = np.full(2, -np.inf)

    np.divide(
        boundary - origin, ray,
        out=entries,
        where=np.abs(ray) > 1e-9
    )

    if abs(entries[0] - entries[1]) < 0.001:
        return None

    axis = int(np.argmax(entries))

    return axis, float(boundary[axis]), np.sign(ray[axis])


# ==========================================================
# 3. IR LOCALIZATION AND OBSTACLE DETECTION
# ==========================================================

def position_from_ir_sensors(
    pos_cam=(0.1,0.1), theta_cam=0,
    pos_enc=None, theta_enc=None,
    obstacle_map= [
            [1, 1, 1, 1],
            [1, 1, 1, 1],
            [1, 1, 1, 1],
            [1, 1, 1, 1]
        ], resolution=0.02, node=None
):
    # Select reference pose.
    if valid_pose(pos_cam, theta_cam):
        pos_ref, theta_ref = pos_cam, theta_cam
    elif valid_pose(pos_enc, theta_enc):
        pos_ref, theta_ref = pos_enc, theta_enc
    else:
        return None, None, None

    pos_ref = np.asarray(pos_ref, dtype=float)
    obstacle_map = np.asarray(obstacle_map)

    if (
        obstacle_map.ndim != 2
        or not np.isfinite(resolution)
        or resolution <= 0
    ):
        return None, None, None

    R = rotation_matrix(theta_ref)

    try:
        raw = np.asarray(node["prox.horizontal"], dtype=float)

        if raw.ndim != 1 or not np.all(np.isfinite(raw[SENSOR_IDS])):
            return None, None, None

    except (KeyError, IndexError, TypeError, ValueError):
        return None, None, None

    obstacles_ir = []
    walls = {}

    # ------------------------------------------------------
    # A. Detect all obstacles and associate known walls
    # ------------------------------------------------------

    for i in SENSOR_IDS:

        if raw[i] < IR_THRESHOLD:
            continue

        d = ir_to_distance(i, raw[i])
        alpha = IR_ANGLES[i]

        origin = pos_ref + R @ SENSOR_OFFSETS[i]
        angle = theta_ref + alpha

        expected, cell = distance_to_wall(
            origin, angle, obstacle_map, resolution
        )

        point = (
            SENSOR_OFFSETS[i] + d * direction(alpha)
            if d is not None else None
        )

        obstacle_pos = (
            pos_ref + R @ point
            if point is not None else None
        )

        # Store every detection, including unknown obstacles.
        obstacles_ir.append({
            "sensor_id": i,
            "distance": d,
            "position": obstacle_pos,
            "expected_distance": expected
        })

        # Only compatible obstacles can be used for localization.
        tolerance = DISTANCE_TOLERANCE[i] + MAP_TOLERANCE

        if (
            d is None or expected is None
            or expected <= 0
            or abs(d - expected) > tolerance
        ):
            continue

        wall = identify_wall(origin, angle, cell, resolution)

        if wall is not None:
            walls.setdefault(wall, []).append((i, point, d))

    if not walls:
        return None, None, obstacles_ir

    # ------------------------------------------------------
    # B. Estimate robot orientation
    # ------------------------------------------------------

    angles = []

    for (axis, coordinate, sign), measurements in walls.items():

        if len(measurements) < 2:
            continue

        P = np.array([p for i, p, d in measurements])
        w = np.array([IR_WEIGHTS[i] for i, p, d in measurements])

        centered = P - np.average(P, axis=0, weights=w)

        _, singular_values, vh = np.linalg.svd(
            centered * np.sqrt(w[:, None])
        )

        if singular_values[0] < 0.001:
            continue

        normal = np.zeros(2)
        normal[axis] = sign

        local_normal = vh[-1]

        if np.dot(R @ local_normal, normal) < 0:
            local_normal = -local_normal

        theta = normalize_angle(
            np.arctan2(normal[1], normal[0])
            - np.arctan2(local_normal[1], local_normal[0])
        )

        if abs(normalize_angle(theta - theta_ref)) <= ANGLE_RANGE:
            angles.append(theta)

    theta_ir = (
        np.arctan2(
            np.mean(np.sin(angles)),
            np.mean(np.cos(angles))
        )
        if angles else None
    )

    # ------------------------------------------------------
    # C. Estimate robot position
    # ------------------------------------------------------

    theta = theta_ir if theta_ir is not None else theta_ref
    R = rotation_matrix(theta)

    pos_ir = pos_ref.copy()

    for axis in (0, 1):

        values, weights = [], []

        for (wall_axis, coordinate, sign), measurements in walls.items():

            if wall_axis != axis:
                continue

            for i, point, d in measurements:
                values.append(coordinate - (R @ point)[axis])
                weights.append(IR_WEIGHTS[i])

        if values:
            pos_ir[axis] = np.average(values, weights=weights)

    # ------------------------------------------------------
    # D. Validate IR localization
    # ------------------------------------------------------

    if (
        np.linalg.norm(pos_ir - pos_ref) > SEARCH_RADIUS
        or not cell_is_free(pos_ir, obstacle_map, resolution)
    ):
        return None, None, obstacles_ir

    for wall, measurements in walls.items():

        for i, point, d in measurements:

            origin = pos_ir + R @ SENSOR_OFFSETS[i]
            angle = theta + IR_ANGLES[i]

            if not cell_is_free(origin, obstacle_map, resolution):
                return None, None, obstacles_ir

            expected, cell = distance_to_wall(
                origin, angle, obstacle_map, resolution
            )

            if (
                expected is None
                or abs(expected - d) > DISTANCE_TOLERANCE[i]
                or identify_wall(origin, angle, cell, resolution) != wall
            ):
                return None, None, obstacles_ir

    return pos_ir, theta_ir, obstacles_ir
