import logging
import os
from typing import Any

from debugging import System, logger
from types_and_constants import (
    DEBUG,
    DEGREE_IN_RAD,
    DIAGONAL_MAX_DIST_IF_WALL1,
    DIAGONAL_MAX_DIST_IF_WALL2,
    ORTOGONAL_MAX_DIST_IF_WALL,
    PI,
    QUADRANT_OF_DELTA,
    TARGET_COORDINATE_BY_MOVE_ANGLE,
    WALL_FROM_ROBOT_POSITION,
    Coordinate,
    Numeric,
    Quadrant,
    Side,
    SideDirection,
)

WebotsRobot = Any


def delay(
    robot: WebotsRobot,
    time_ms: Numeric,
    time_step: int = int(os.getenv("TIME_STEP", 32)),
):
    logger.debug(f"Esperando: {time_ms}ms", System.delay)

    init_time = robot.getTime()
    while robot.step(time_step) != -1:
        time_elapsed_ms = (robot.getTime() - init_time) * 1000.0
        if time_elapsed_ms > time_ms:
            break


def round_if_almost_0(value: float):
    return 0 if round(value, 2) == 0 else value


def cyclic_angle(angle: Numeric) -> Numeric:
    """
    It may be used for cases in which angles are cyclic and should
    be considered in range[0; 2*PI].
    Example of returns depending on the `angle`:
    - `PI` => returns `PI`
    - `-0.5 * PI` => returns `1.5 * PI`, because it is turning clockwise
    this angle, so it is 0.5 * PI before a complete turn (2 * PI), then
    it is `2.0 * PI - 0.5 * PI = 1.5 * PI`
    - `5.5 * PI` => returns `1.5 * PI`, because 3.5 * PI, is 2 complete
    turns + 1.5 * PI

    :return: The `angle` in range [0; 2*PI].
    :rtype: Same type as `angle`.
    """
    while angle < 0:
        angle += 2 * PI
    while angle >= 2 * PI:
        angle -= 2 * PI

    return angle


def cyclic_angle_difference(
    measured_angle: Numeric, expected_angle: Numeric
) -> Numeric:
    """How much should be added to `measured_angle` to reach `expected_angle`,
    considering they are cyclic.

    Note that it may be negative.
    """
    angle_difference = measured_angle - expected_angle
    if abs(angle_difference) >= 180 * DEGREE_IN_RAD:
        if measured_angle < expected_angle:
            measured_angle += 360 * DEGREE_IN_RAD
        else:
            expected_angle += 360 * DEGREE_IN_RAD
        angle_difference = measured_angle - expected_angle
    return angle_difference


def calculate_wall_position(
    robot_position: Coordinate,
    side_direction: SideDirection,
    angle: Numeric,
    wall_idx: int,
    angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2)),
) -> tuple[Coordinate, Side]:
    angle = round(angle, 2)
    equivalent_angle = None
    for testing_angle in WALL_FROM_ROBOT_POSITION[side_direction]:
        if abs(testing_angle - angle) <= angle_max_difference:
            equivalent_angle = testing_angle

    if equivalent_angle is None:
        if DEBUG:
            error_message = (
                f"Inválido {angle=} ao tentar calcular posição da parede "
                f"detectada de {robot_position=} olhando pela {side_direction=}"
            )
            logging.error(error_message)
            logging.debug(
                f"Ângulos disponíveis: {list(WALL_FROM_ROBOT_POSITION[side_direction].keys())}"
            )
            raise ValueError(error_message)

        # Gets the most similar angle categorized
        equivalent_angle = angle
        for testing_angle in WALL_FROM_ROBOT_POSITION[side_direction]:
            if abs(testing_angle - angle) < abs(equivalent_angle - angle):
                equivalent_angle = testing_angle

    delta_from_robot_position, side = WALL_FROM_ROBOT_POSITION[side_direction][
        equivalent_angle
    ][wall_idx]
    quarter_tile = robot_position + delta_from_robot_position

    return quarter_tile, side


def coordinate_after_move(
    position: Coordinate,
    angle: Numeric,
    angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2)),
) -> Coordinate:
    angle = round(angle, 2)
    for target_angle in TARGET_COORDINATE_BY_MOVE_ANGLE:
        if abs(target_angle - angle) <= angle_max_difference:
            return position + TARGET_COORDINATE_BY_MOVE_ANGLE[target_angle]

    if DEBUG:
        error_message = (
            f"Inválido {angle=} ao tentar pegar coordenada "
            f"após movimentação começando de {position}"
        )
        logging.error(error_message)
        logging.debug(
            f"Ângulos disponíveis: {list(TARGET_COORDINATE_BY_MOVE_ANGLE.keys())}"
        )
        raise ValueError(error_message)

    # Gets the most similar angle categorized
    most_similar_angle = (angle, position + TARGET_COORDINATE_BY_MOVE_ANGLE[0])
    for testing_angle, delta in TARGET_COORDINATE_BY_MOVE_ANGLE.items():
        if abs(testing_angle - angle) < most_similar_angle[0]:
            most_similar_angle = (testing_angle, position + delta)

    return most_similar_angle[1]


def tile_pos_with_quarter_tile(quarter_tile_pos: Coordinate) -> Coordinate:
    """
    :return: The `Coordinate` of tile that contains this quarter tile.
    """
    return Coordinate(quarter_tile_pos.x // 2, (quarter_tile_pos.y + 1) // 2)


def quarter_tile_quadrant(quarter_tile_pos: Coordinate) -> Quadrant:
    """
    :return: The quadrant that the quarter tile is corresponding
    to the tile in which it is located.
    """
    delta = quarter_tile_pos - tile_pos_with_quarter_tile(quarter_tile_pos) * 2
    return QUADRANT_OF_DELTA[delta]


def side_angle_from_map_angle(map_angle: float, robot_rotation_angle: float) -> float:
    """
    Returns the angle of the side of the robot that is in the same direction
    of the `map_angle`.
    """
    return cyclic_angle(map_angle - robot_rotation_angle)


def get_blocking_wall(wall_distance: float, delta_angle_in_degree: int) -> int:
    if abs(delta_angle_in_degree) % 90 == 0:
        wall_blocking = 0 if wall_distance <= ORTOGONAL_MAX_DIST_IF_WALL else -1
    elif abs(delta_angle_in_degree) % 45 == 0 and delta_angle_in_degree != 0:
        wall_blocking = (
            0
            if wall_distance <= DIAGONAL_MAX_DIST_IF_WALL1
            else (1 if wall_distance <= DIAGONAL_MAX_DIST_IF_WALL2 else -1)
        )
    return wall_blocking


def get_central_blocking_wall(wall_distance: float, delta_angle_in_degree: int) -> int:
    if abs(delta_angle_in_degree) % 90 != 0:
        return -1
    return 0 if wall_distance <= ORTOGONAL_MAX_DIST_IF_WALL else -1
