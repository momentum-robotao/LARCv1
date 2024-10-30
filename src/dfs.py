import os

from debugging import DebugInfo, System
from helpers import (
    calculate_wall_position,
    coordinate_after_move,
    cyclic_angle,
    get_blocking_wall,
    get_central_blocking_wall,
    side_angle_from_map_angle,
)
from maze import Maze
from robot import Robot
from types_and_constants import (
    DEBUG,
    DEGREE_IN_RAD,
    QUADRANT_OF_DELTA,
    TILE_SIZE,
    AreaDFSMappable,
    Coordinate,
    MovementResult,
    WallColisionError,
)


def adjust_wall_distance(
    robot: Robot, angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2))
) -> None:
    return


def dfs(
    position: Coordinate,
    maze: Maze,
    robot: Robot,
    debug_info: DebugInfo,
    area: AreaDFSMappable,
    starting: bool = False,
):
    """
    Map all the specified `area`, then go to transition `area+1`.
    Robot position is specified by the lower left coordinate.

    :return: The final position of the robot.
    """
    if DEBUG:
        debug_info.send(f"Começando DFS em {position=} da {area=}", System.dfs_state)
    adjust_wall_distance(robot)
    maze.mark_visited(position)

    robot.step()
    start_angle = robot.imu.get_rotation_angle()
    if DEBUG:
        debug_info.send(
            f"DFS de {position=} começou com {start_angle=}rad", System.dfs_verification
        )

    # Transition to neighbours on grid, prioritizing front, left and right
    # before diagonals
    for delta_angle_in_degree in [0, -90, 90, 45, -45] + (
        [180, 135, -135] if starting else []
    ):  # -45, 45] + (
        #     [135, 180, -135]
        #     if starting
        #     else []
        #     # ? Only in the start we can't assume that backward empty: important to make sure
        #     # everything was visited and all walls mapped
        # ):
        delta_angle = delta_angle_in_degree * DEGREE_IN_RAD

        movement_angle = cyclic_angle(start_angle + delta_angle)
        movement_side_angle = side_angle_from_map_angle(
            movement_angle, robot.imu.get_rotation_angle()
        )
        new_robot_position = coordinate_after_move(position, movement_angle)

        if DEBUG:
            debug_info.send(
                f"DFS de {position=}, olhando vizinho de ângulo:\n"
                f"- {delta_angle_in_degree}° em relação ao ângulo inicial da DFS\n"
                f"- {movement_angle}rad de direção em relação ao mapa\n"
                f"- {movement_side_angle}rad em relação à frente do robô.\n"
                f"Esse vizinho está na coordenada {new_robot_position}",
                System.dfs_state,
            )

        # For each side of the tiles that would be used to go to new position,
        # map if there is a wall there or not
        left_wall_angle = cyclic_angle(movement_side_angle - 20 * DEGREE_IN_RAD)
        right_wall_angle = cyclic_angle(movement_side_angle + 20 * DEGREE_IN_RAD)

        left_wall_distance = robot.lidar.get_side_distance(left_wall_angle)
        right_wall_distance = robot.lidar.get_side_distance(right_wall_angle)
        central_wall_distance = robot.lidar.get_side_distance(movement_side_angle)

        if DEBUG:
            debug_info.send(
                f"Distância das paredes no caminho para essa nova posição: {left_wall_distance=}m, "
                f"{right_wall_distance=}m e {central_wall_distance=}m",
                System.dfs_verification,
            )

        left_wall_blocking = get_blocking_wall(
            left_wall_distance, delta_angle_in_degree
        )
        right_wall_blocking = get_blocking_wall(
            right_wall_distance, delta_angle_in_degree
        )
        central_wall_blocking = get_central_blocking_wall(
            central_wall_distance, delta_angle_in_degree
        )

        # blocked? don't move and map these walls
        if DEBUG:
            debug_info.send(
                f"Paredes detectadas: {left_wall_blocking=}; {right_wall_blocking=} "
                f"e {central_wall_blocking=}",
                System.dfs_verification,
            )
        if left_wall_blocking != -1:
            quarter_tile, side = calculate_wall_position(
                position, "front_left", movement_angle, left_wall_blocking
            )
            maze.add_wall(quarter_tile, side)
        if right_wall_blocking != -1:
            quarter_tile, side = calculate_wall_position(
                position, "front_right", movement_angle, right_wall_blocking
            )
            maze.add_wall(quarter_tile, side)
        if left_wall_blocking != -1 or right_wall_blocking != -1:
            if DEBUG:
                debug_info.send(
                    "Há paredes no caminho para o vizinho: não será visitado",
                    System.dfs_decision,
                )
            continue

        if (
            central_wall_blocking != -1
            and right_wall_blocking == -1
            and left_wall_blocking == -1
        ):
            quarter_tile, side = calculate_wall_position(
                position, "front_center", movement_angle, central_wall_blocking
            )
            maze.add_wall(quarter_tile, side)
            if DEBUG:
                debug_info.send(
                    "Há parede central no caminho para o vizinho: não será visitado",
                    System.dfs_decision,
                )
            continue

        # visited? don't move
        if all(
            maze.is_visited(new_robot_position + delta)
            for delta in QUADRANT_OF_DELTA.keys()
        ):
            if DEBUG:
                debug_info.send(
                    "Vizinho já foi visitado: não será visitado novamente",
                    System.dfs_decision,
                )
            continue

        # otherwise, visit it recursively (with dfs)
        if DEBUG:
            debug_info.send("Movendo para o vizinho", System.dfs_decision)
        new_position_distance = (
            TILE_SIZE / 2 * (1.44 if delta_angle_in_degree in [45, -45] else 1)
        )
        robot.motor.rotate_to_angle(movement_angle, robot.imu)
        try:
            movement_result = robot.motor.move(
                "forward",
                robot.gps,
                robot.lidar,
                robot.color_sensor,
                new_position_distance,
            )
            if movement_result == MovementResult.left_right_hole:
                continue
            elif movement_result == MovementResult.moved:
                pass
            elif DEBUG:
                debug_info.send(
                    f"Resultado inesperado para movimento: {movement_result}",
                    System.unknown_error,
                    "error",
                )
                raise Exception(f"Resultado de movimento inesperado: {movement_result}")
        except WallColisionError:
            continue

        dfs(new_robot_position, maze, robot, debug_info, area)

        if DEBUG:
            debug_info.send("Retornando do vizinho", System.dfs_decision)
        robot.motor.move(
            "backward",
            robot.gps,
            robot.lidar,
            robot.color_sensor,
            new_position_distance,
        )

    if DEBUG:
        debug_info.send(
            f"Finalizando DFS de {position=}. Voltando para {start_angle=}",
            System.dfs_decision,
        )

    # ? important to ensure that when last dfs move backward with robot
    # it is in the same direction and will properly "undo" the movement to
    # this tile, coming back to the last tile.
    robot.motor.rotate_to_angle(start_angle, robot.imu)
