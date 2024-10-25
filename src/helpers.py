import logging
import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
from types_and_constants import (
    DEBUG,
    DELTA_TO_QUADRANT,
    DIAGONAL_MAX_DIST_IF_WALL1,
    DIAGONAL_MAX_DIST_IF_WALL2,
    ORTOGONAL_MAX_DIST_IF_WALL,
    PI,
    TARGET_COORDINATE,
    WALL_FROM_ROBOT_POSITION,
    Coordinate,
    Numeric,
    Quadrant,
    RobotQuadrant,
    Side,
)


def delay(
    robot: WebotsRobot,
    debug_info: DebugInfo,
    time_ms: Numeric,
    time_step: int = int(os.getenv("TIME_STEP", 32)),
):
    if DEBUG:
        debug_info.send(f"Esperando: {time_ms}ms", System.delay)

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


# TODO: rename RobotQuadrant as it has "front_center" now
# TODO: merge with `coordinate_after_move` to get for example the most similar rotation angle
def calculate_wall_position(
    robot_position: Coordinate,
    robot_quadrant: RobotQuadrant,
    angle: Numeric,
    wall_idx: int,
    angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2)),
) -> tuple[Coordinate, Side]:
    equivalent_rotation_angle = None
    equivalent_rotation_angle = round(angle, 2)
    for testing_angle in WALL_FROM_ROBOT_POSITION[robot_quadrant]:
        if abs(testing_angle - equivalent_rotation_angle) <= angle_max_difference:
            equivalent_rotation_angle = testing_angle

    if equivalent_rotation_angle is None:
        if DEBUG:
            error_message = (
                f"Inválido {equivalent_rotation_angle=} ao tentar calcular posição da parede "
                f"detectada de {robot_position=} olhando pelo {robot_quadrant=}"
            )
            logging.error(error_message)
            logging.debug(
                f"Ângulos disponíveis: {list(WALL_FROM_ROBOT_POSITION[robot_quadrant].keys())}"
            )
            raise ValueError(error_message)

        # Gets the most similar angle categorized
        equivalent_rotation_angle = equivalent_rotation_angle
        for testing_angle in WALL_FROM_ROBOT_POSITION[robot_quadrant]:
            if (
                abs(testing_angle - equivalent_rotation_angle)
                < equivalent_rotation_angle
            ):
                equivalent_rotation_angle = testing_angle

    delta_from_robot_position, side = WALL_FROM_ROBOT_POSITION[robot_quadrant][
        equivalent_rotation_angle
    ][wall_idx]
    quarter_tile = robot_position + delta_from_robot_position
    return quarter_tile, side


def coordinate_after_move(
    position: Coordinate,
    angle: Numeric,
    angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2)),
) -> Coordinate:
    angle = round(angle, 2)
    for target_angle in TARGET_COORDINATE:
        if abs(target_angle - angle) <= angle_max_difference:
            return position + TARGET_COORDINATE[target_angle]

    if DEBUG:
        error_message = (
            f"Inválido {angle=} ao tentar pegar coordenada "
            f"após movimentação começando de {position}"
        )
        logging.error(error_message)
        logging.debug(f"Ângulos disponíveis: {list(TARGET_COORDINATE.keys())}")
        raise ValueError(error_message)

    # Gets the most similar angle categorized
    most_similar_angle = (angle, position + Coordinate(0, 1))
    for reference_angle, delta in TARGET_COORDINATE.items():
        if abs(reference_angle - angle) < most_similar_angle[0]:
            most_similar_angle = (reference_angle, position + delta)

    return most_similar_angle[1]


def tile_pos_with_quarter_tile(quarter_tile_pos: Coordinate) -> Coordinate:
    """
    :return: The `Coordinate` of tile that contains this quarter tile.
    """
    return Coordinate(quarter_tile_pos.x // 2, quarter_tile_pos.y // 2)


def quarter_tile_quadrant(quarter_tile_pos: Coordinate) -> Quadrant:
    """
    :return: The quadrant that the quarter tile is corresponding
    to the tile in which it is located.
    """
    delta = quarter_tile_pos - tile_pos_with_quarter_tile(quarter_tile_pos) * 2
    return DELTA_TO_QUADRANT[delta]


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
