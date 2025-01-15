from debugging import RobotLogger
from dfs import dfs
from maze import Maze
from robot import Robot
from types_and_constants import SLOW_DOWN_DIST, Coordinate, SpecialTileType


def solve_map(robot: Robot, logger: RobotLogger, maze: Maze) -> None:
    initial_position = Coordinate(0, 0)
    maze.set_tile_type(initial_position, SpecialTileType.STARTING)
    transitions = dfs(
        initial_position,
        maze,
        robot,
        logger,
        area=1,
        moves_before=[],
        starting=True,
    )
    best_transition_pos, best_transition, best_moves_before = None, None, None
    priority = [
        SpecialTileType.PASSAGE_2_4,
        SpecialTileType.PASSAGE_3_4,
        SpecialTileType.PASSAGE_1_4,
        None,
    ]
    # print(transitions)
    for transition_pos, transition, moves_before in transitions:
        if priority.index(transition) < priority.index(best_transition):
            best_transition_pos, best_transition, best_moves_before = (
                transition_pos,
                transition,
                moves_before,
            )
    # print(f"Melhor: {best_transition}")
    if best_transition is None:
        return

    # go to room 4
    for move in best_moves_before:
        command, args = move
        if command == "rotate_to_angle":
            robot.rotate_to_angle(args[0])
        if command == "move":
            robot.move(
                args[0],
                Maze(logger),
                dist=args[1],
                slow_down_dist=SLOW_DOWN_DIST / 3,
            )

    dfs(
        best_transition_pos,
        maze,
        robot,
        logger,
        area=4,
        moves_before=[],
        starting=True,
    )

    # TODO4: retornar pra tile inicial (+10%)
    # best_moves_before.reverse()
    # for move in best_moves_before:
    #     command, args = move
    #     if command == "rotate_to_angle":
    #         robot.rotate_to_angle(args[0])
    #     if command == "move":
    #         robot.move(
    #             "backward",
    #             Maze(logger),
    #             dist=args[1],
    #             slow_down_dist=SLOW_DOWN_DIST / 3,
    #         )
