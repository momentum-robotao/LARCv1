from debugging import System, logger
from maze import Maze
from robot import (
    Move,
    MovementResult,
    Robot,
    Rotate,
    create_movement_velocity_controller,
)
from types_and_constants import (
    DEBUG,
    DEGREE_IN_RAD,
    DEGREES_TO_ENSURE_VIEW_WT,
    QUADRANT_OF_DELTA,
    SLOW_DOWN_DIST,
    TILE_SIZE,
    AreaDFSMappable,
    Coordinate,
    LackOfProgressError,
    SpecialTileType,
)
from utils import (
    calculate_wall_position,
    coordinate_after_move,
    cyclic_angle,
    get_blocking_wall,
    get_central_blocking_wall,
    side_angle_from_map_angle,
    tile_pos_with_quarter_tile,
)


def dfs(
    position: Coordinate,
    maze: Maze,
    robot: Robot,
    area: AreaDFSMappable,
    moves_before: list[tuple[str, tuple]],
    starting: bool = False,
) -> list[tuple[Coordinate, SpecialTileType, list[tuple[str, tuple]]]]:
    """
    Map all the specified `area`, then return transitions to next area.
    Robot position is specified by the lower left coordinate.
    """

    if DEBUG:
        answer_map = "\n".join("".join(line) for line in maze.get_answer_maze())
        logger.debug(answer_map, System.maze_answer_str)
    transitions: list[tuple[Coordinate, SpecialTileType, list[tuple[str, tuple]]]] = []

    logger.info(f"Começando DFS em {position=} da {area=}", System.dfs_state)

    start_angle = robot.expected_angle
    logger.info(
        f"DFS de {position=} começou com {start_angle=}rad", System.dfs_verification
    )

    maze.mark_visited(position)
    robot.step()

    # TODO-: swamp etc podem estar acessíveis apenas em certo quarter tile
    colored_tile = None
    if position.x % 2 == 0 and position.y % 2 == 0:  # centered in tile
        colored_tile = robot.color_sensor.check_colored_special_tile()
        if area == 4 and colored_tile not in [
            SpecialTileType.PASSAGE_1_4,
            SpecialTileType.PASSAGE_2_4,
            SpecialTileType.PASSAGE_3_4,
        ]:
            maze.set_tile_type(position, SpecialTileType.AREA_4)
    if colored_tile and area != 4:
        logger.info(f"Cor do chão detectou {colored_tile}", System.check_tile_color)
        maze.set_tile_type(position, colored_tile)
    else:
        logger.info(f"Não detectou cor em {position}", System.check_tile_color)

    if colored_tile in [
        SpecialTileType.PASSAGE_1_4,
        SpecialTileType.PASSAGE_2_4,
        SpecialTileType.PASSAGE_3_4,
    ] and (area != 4 or starting is False):
        return [(position, colored_tile, moves_before)]

    # Transition to neighbours on grid, prioritizing front, left and right
    # before diagonals
    for delta_angle_in_degree in [0, 90, -90] + (
        [180] if starting and area != 4 else []
    ):
        #     # ? Only in the start we can't assume that backward empty: important to make sure
        #     # everything was visited and all walls mapped
        # ):
        delta_angle = delta_angle_in_degree * DEGREE_IN_RAD

        movement_angle = cyclic_angle(start_angle + delta_angle)
        movement_side_angle = side_angle_from_map_angle(
            movement_angle, robot.imu.get_rotation_angle()
        )
        new_robot_position = coordinate_after_move(position, movement_angle)

        logger.info(
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

        logger.info(
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
        logger.info(
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
            logger.info(
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
            logger.info(
                "Há parede central no caminho para o vizinho: não será visitado",
                System.dfs_decision,
            )
            continue

        # visited? don't move
        if all(
            maze.is_visited(new_robot_position + delta)
            for delta in QUADRANT_OF_DELTA.keys()
        ):
            logger.info(
                "Vizinho já foi visitado: não será visitado novamente",
                System.dfs_decision,
            )
            continue

        # otherwise, visit it recursively (with dfs)
        logger.info("Movendo para o vizinho", System.dfs_decision)
        new_position_distance = (
            TILE_SIZE / 2 * (1.44 if delta_angle_in_degree in [45, -45] else 1)
        )
        viz_moves = moves_before.copy()
        viz_moves.append(("rotate_to_angle", (movement_angle,)))
        robot.run(Rotate("fastest", movement_angle, maze=maze))
        if delta_angle_in_degree == 90:
            robot.run(
                Rotate(
                    "right",
                    DEGREES_TO_ENSURE_VIEW_WT * DEGREE_IN_RAD,
                    maze=maze,
                    correction_rotation=True,
                )
            )
            robot.run(
                Rotate(
                    "left",
                    DEGREES_TO_ENSURE_VIEW_WT * DEGREE_IN_RAD,
                    maze=maze,
                    correction_rotation=True,
                )
            )
        if delta_angle_in_degree == -90:
            robot.run(
                Rotate(
                    "left",
                    DEGREES_TO_ENSURE_VIEW_WT * DEGREE_IN_RAD,
                    maze=maze,
                    correction_rotation=True,
                )
            )
            robot.run(
                Rotate(
                    "right",
                    DEGREES_TO_ENSURE_VIEW_WT * DEGREE_IN_RAD,
                    maze=maze,
                    correction_rotation=True,
                )
            )

        viz_moves.append(("move", ("forward", new_position_distance)))
        movement_result = robot.run(
            Move(
                "forward",
                new_position_distance,
                maze=maze,
                speed_controller=create_movement_velocity_controller(
                    slow_down_dist=SLOW_DOWN_DIST / 3
                ),
            )
        )

        if movement_result in [
            MovementResult.central_hole,
            MovementResult.left_hole,
            MovementResult.right_hole,
            MovementResult.unavoidable_obstacle,
        ]:
            logger.info("Blocked: return to initial position", System.movement_reason)
            comeback_result = robot.run(
                Move(
                    "backward",
                    0,  # ? Move just to correct the last move, which was unsucessful
                    maze=maze,
                    speed_controller=create_movement_velocity_controller(
                        slow_down_dist=SLOW_DOWN_DIST / 3
                    ),
                )
            )
            if comeback_result != MovementResult.moved:
                raise LackOfProgressError()

        angle_to_hole = None
        match movement_result:
            case MovementResult.moved:
                pass

            case MovementResult.central_hole:
                angle_to_hole = movement_angle
            case MovementResult.left_hole:
                angle_to_hole = cyclic_angle(movement_angle - 45 * DEGREE_IN_RAD)
            case MovementResult.right_hole:
                angle_to_hole = cyclic_angle(movement_angle + 45 * DEGREE_IN_RAD)

            case MovementResult.unavoidable_obstacle:
                continue

            case _:
                if DEBUG:
                    logger.error(
                        f"Resultado inesperado para movimento: {movement_result}",
                        System.unknown_error,
                    )
                    raise Exception(
                        f"Resultado de movimento inesperado: {movement_result}"
                    )

        if angle_to_hole is not None:
            robot_position_on_hole = coordinate_after_move(
                coordinate_after_move(position, angle_to_hole), movement_angle
            )
            maze.set_tile_type(robot_position_on_hole, SpecialTileType.HOLE)
            hole_tile_position = tile_pos_with_quarter_tile(robot_position_on_hole)
            maze.mark_visited_tile(hole_tile_position)
            continue

        transitions.extend(
            dfs(
                new_robot_position,
                maze,
                robot,
                area,
                moves_before=viz_moves,
            )
        )

        logger.info("Retornando do vizinho", System.dfs_decision)
        comeback_result = robot.run(
            Move(
                "backward",
                new_position_distance,
                maze=maze,
                speed_controller=create_movement_velocity_controller(
                    slow_down_dist=SLOW_DOWN_DIST / 3
                ),
            )
        )
        if comeback_result != MovementResult.moved:
            raise LackOfProgressError()

    logger.info(
        f"Finalizando DFS de {position=}. Voltando para {start_angle=}",
        System.dfs_decision,
    )

    # ? important to ensure that when last dfs move backward with robot
    # it is in the same direction and will properly "undo" the movement to
    # this tile, coming back to the last tile.
    robot.step()
    robot.run(Rotate("fastest", start_angle, maze=maze))

    if DEBUG:
        answer_map = "\n".join("".join(line) for line in maze.get_answer_maze())
        logger.debug(answer_map, System.maze_answer_str)

    return transitions
