
from position_from_ir_sensors import (
    DISTANCE_TOLERANCE,
    MAP_TOLERANCE
)


def obstacle_is_known(obstacle):
    i = obstacle["sensor_id"]
    d = obstacle["distance"]
    expected = obstacle["expected_distance"]

    return bool(
        d is not None
        and expected is not None
        and expected > 0
        and abs(d - expected)
        <= DISTANCE_TOLERANCE[i] + MAP_TOLERANCE
    )


def path_clear(obstacles_ir):
    return (
        obstacles_ir is not None
        and all(obstacle_is_known(o) for o in obstacles_ir)
    )


def get_unknown_obstacles(obstacles_ir):
    return [
        o for o in (obstacles_ir or [])
        if not obstacle_is_known(o)
    ]
