from debugging import DebugInfo, System
from helpers import (
    calculate_wall_position,
    coordinate_after_move,
    cyclic_angle,
    cyclic_angle_difference,
    get_blocking_wall,
    get_central_blocking_wall,
    side_angle_from_map_angle,
    tile_pos_with_quarter_tile,
)
from maze import Maze
from robot import Robot
from types_and_constants import (
    DEBUG,
    DEGREE_IN_RAD,
    EXPECTED_WALL_DISTANCE,
    QUADRANT_OF_DELTA,
    SLOW_DOWN_DIST,
    TILE_SIZE,
    AreaDFSMappable,
    Coordinate,
    MovementResult,
    SpecialTileType,
    WallColisionError,
)


def get_errors(robot: Robot, field_of_view: float):
    y_error = 0.0
    if robot.lidar.has_wall("back", field_of_view=field_of_view, use_min=True):
        y_error = EXPECTED_WALL_DISTANCE - robot.lidar.get_side_distance(
            "back", field_of_view=field_of_view, use_min=True
        )
    elif robot.lidar.has_wall("front", field_of_view=field_of_view, use_min=True):
        y_error = (
            robot.lidar.get_side_distance(
                "front", field_of_view=field_of_view, use_min=True
            )
            - EXPECTED_WALL_DISTANCE
        )

    x_error = 0.0
    if robot.lidar.has_wall("right", field_of_view=field_of_view, use_min=True):
        x_error = EXPECTED_WALL_DISTANCE - robot.lidar.get_side_distance(
            "right",
            use_min=True,
            field_of_view=field_of_view,
        )
    elif robot.lidar.has_wall("left", field_of_view=field_of_view, use_min=True):
        x_error = (
            robot.lidar.get_side_distance(
                "left", field_of_view=field_of_view, use_min=True
            )
            - EXPECTED_WALL_DISTANCE
        )

    angle_error = cyclic_angle_difference(
        robot.imu.get_rotation_angle(), robot.expected_angle
    )
    return y_error, x_error, angle_error


# TODO+: ficar ajustando e desajustando
def adjust_wall_distance(
    robot: Robot,
    debug_info: DebugInfo,
    maze: Maze,
    angle_max_error: float = 2 * DEGREE_IN_RAD,
    field_of_view: float = 30 * DEGREE_IN_RAD,
    wall_max_y_error: float = EXPECTED_WALL_DISTANCE / 5,
    wall_max_x_error: float = EXPECTED_WALL_DISTANCE / 3,
) -> None:
    y_error, x_error, angle_error = get_errors(robot, field_of_view)

    # Obstacle
    if (
        robot.lidar.get_side_distance("left", field_of_view=field_of_view, use_min=True)
        + robot.lidar.get_side_distance(
            "right", field_of_view=field_of_view, use_min=True
        )
        <= 0.03
        or robot.lidar.get_side_distance(
            "front", field_of_view=field_of_view, use_min=True
        )
        + robot.lidar.get_side_distance(
            "back", field_of_view=field_of_view, use_min=True
        )
        <= 0.03
    ):
        print("não ajusta: obstáculo")
        return

    if angle_error <= -angle_max_error:
        robot.rotate("right", abs(angle_error), correction_rotation=True)
        y_error, x_error, angle_error = get_errors(robot, field_of_view)
    if angle_error >= angle_max_error:
        robot.rotate("left", angle_error, correction_rotation=True)
        y_error, x_error, angle_error = get_errors(robot, field_of_view)

    if y_error <= -wall_max_y_error:
        robot.move(
            "backward",
            maze,
            dist=abs(y_error),
            correction_move=True,
        )
        y_error, x_error, angle_error = get_errors(robot, field_of_view)
    if y_error >= wall_max_y_error:
        robot.move(
            "forward",
            maze,
            dist=y_error,
            correction_move=True,
        )
        y_error, x_error, angle_error = get_errors(robot, field_of_view)

    if x_error <= -wall_max_x_error:
        robot.rotate_90_left()
        robot.move(
            "backward",
            maze,
            dist=abs(x_error),
            correction_move=True,
        )
        robot.rotate_90_right()
    if x_error >= wall_max_x_error:
        robot.rotate_90_left()
        robot.move(
            "forward",
            maze,
            dist=x_error,
            correction_move=True,
        )
        robot.rotate_90_right()


# TODO+: parede quarter tile ou canto de quarter já vira
def alley(robot: Robot, maze: Maze, position: Coordinate, start_angle: float) -> bool:
    for delta_angle_in_degree, side in [(0, "front"), (90, "right"), (-90, "left")]:
        delta_angle = delta_angle_in_degree * DEGREE_IN_RAD

        movement_angle = cyclic_angle(start_angle + delta_angle)
        new_robot_position = coordinate_after_move(position, movement_angle)

        if any(
            not maze.is_visited(new_robot_position + delta)
            for delta in QUADRANT_OF_DELTA.keys()
        ) and not robot.lidar.has_wall(
            side  # type: ignore[arg-type]
        ):
            return False
    return True


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
    print(
        f"DFS {position=}; {robot.expected_position}; {robot.expected_angle / DEGREE_IN_RAD}"
    )
    if DEBUG:
        debug_info.send(f"Começando DFS em {position=} da {area=}", System.dfs_state)

    start_angle = robot.expected_angle
    if DEBUG:
        debug_info.send(
            f"DFS de {position=} começou com {start_angle=}rad", System.dfs_verification
        )

    maze.mark_visited(position)
    robot.step()
    adjust_wall_distance(robot, debug_info, maze)
    robot.recognize_wall_token()

    # TODO-: swamp etc podem estar acessíveis apenas em certo quarter tile
    colored_tile = None
    if position.x % 2 == 0 and position.y % 2 == 0:  # centered in tile
        colored_tile = robot.color_sensor.check_colored_special_tile()
    if colored_tile:
        if DEBUG:
            debug_info.send(
                f"Cor do chão detectou {colored_tile}", System.check_tile_color
            )
        maze.set_tile_type(position, colored_tile)
    else:
        if DEBUG:
            debug_info.send(f"Não detectou cor em {position}", System.check_tile_color)

    if alley(robot, maze, position, start_angle):
        adjust_wall_distance(
            robot, debug_info, maze, wall_max_x_error=0.1, wall_max_y_error=0.1
        )
        robot.rotate_180()

    # Transition to neighbours on grid, prioritizing front, left and right
    # before diagonals
    for delta_angle_in_degree in [0, 90, -90] + ([180] if starting else []):
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
        central_wall_distance = robot.lidar.get_side_distance(
            movement_side_angle, field_of_view=20 * DEGREE_IN_RAD, use_min=True
        )

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
        robot.rotate_to_angle(movement_angle)
        robot.recognize_wall_token()
        try:
            movement_result = robot.move(
                "forward",
                maze,
                dist=new_position_distance,
                slow_down_dist=SLOW_DOWN_DIST / 3,
            )

            angle_to_hole = None
            if movement_result == MovementResult.central_hole:
                angle_to_hole = movement_angle
            elif movement_result == MovementResult.left_hole:
                angle_to_hole = cyclic_angle(movement_angle - 45)
            elif movement_result == MovementResult.right_hole:
                angle_to_hole = cyclic_angle(movement_angle + 45)

            if angle_to_hole is not None:
                robot_position_on_hole = coordinate_after_move(
                    coordinate_after_move(position, angle_to_hole), movement_angle
                )
                maze.set_tile_type(robot_position_on_hole, SpecialTileType.HOLE)
                hole_tile_position = tile_pos_with_quarter_tile(robot_position_on_hole)
                maze.mark_visited_tile(hole_tile_position)
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
        robot.move(
            "backward",
            maze,
            dist=new_position_distance,
            slow_down_dist=SLOW_DOWN_DIST / 3,
        )

    if DEBUG:
        debug_info.send(
            f"Finalizando DFS de {position=}. Voltando para {start_angle=}",
            System.dfs_decision,
        )

    # ? important to ensure that when last dfs move backward with robot
    # it is in the same direction and will properly "undo" the movement to
    # this tile, coming back to the last tile.
    robot.step()
    adjust_wall_distance(robot, debug_info, maze)
    robot.recognize_wall_token()
    robot.rotate_to_angle(start_angle)
