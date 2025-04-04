from dfs import dfs
from maze import Maze
from robot import (
    Move,
    RecognizeWallToken,
    Robot,
    Rotate,
    create_movement_velocity_controller,
)
from types_and_constants import SLOW_DOWN_DIST, Coordinate, SpecialTileType


def solve_map(robot: Robot, maze: Maze) -> None:
    while robot.step() != -1:
        robot.run(RecognizeWallToken(maze))
        from debugging import System, logger
        from utils import delay

        current_position = robot.gps.get_position()
        if wall_tokens := maze.get_wall_tokens_near(current_position):
            logger.info(
                f"Enviará: {wall_tokens}, estão próximos do robô",
                System.wall_token_send,
            )
            robot.motor.stop()
            delay(robot.webots_robot, 1300)
            for wall_token in wall_tokens:
                robot.communicator.send_wall_token_information(
                    current_position, wall_token
                )
        from time import sleep

        sleep(1)

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
