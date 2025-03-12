from dfs import dfs
from maze import Maze
from robot import Move, Robot, Rotate, create_movement_velocity_controller
from types_and_constants import SLOW_DOWN_DIST, Coordinate, SpecialTileType


def solve_map(robot: Robot, maze: Maze) -> None:
    initial_position = Coordinate(0, 0)
    maze.set_tile_type(initial_position, SpecialTileType.STARTING)
    transitions = dfs(
        initial_position,
        maze,
        robot,
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
    for transition_pos, transition, moves_before in transitions:
        if priority.index(transition) < priority.index(best_transition):
            best_transition_pos, best_transition, best_moves_before = (
                transition_pos,
                transition,
                moves_before,
            )
    if best_transition is None:
        return

    # go to room 4
    for move in best_moves_before:
        command, args = move
        if command == "rotate_to_angle":
            robot.run(Rotate("fastest", args[0]))
        if command == "move":
            robot.run(
                Move(
                    args[0],
                    args[1],
                    maze=Maze(),
                    speed_controller=create_movement_velocity_controller(
                        slow_down_dist=SLOW_DOWN_DIST / 3
                    ),
                )
            )

    dfs(  # TODO: improve strategy, refactor(type hint, log...)
        best_transition_pos,
        maze,
        robot,
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
    #             args[1],
    #             Maze(logger),
    #             slow_down_dist=SLOW_DOWN_DIST / 3,
    #         )
