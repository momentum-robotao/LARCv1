from typing import Literal

from debugging import System, logger
from maze import Maze
from recognize_wall_token import reconhece_lado
from types_and_constants import DEGREE_IN_RAD, PI, WallToken
from utils import delay

from .robot import Robot, RobotCommand


class RecognizeWallToken(RobotCommand[bool]):
    def __init__(
        self,
        rotating: bool = False,
        wall_token_approximation: bool = True,
        move_after_found_wall_token: bool = False,
    ) -> None:
        self.rotating = rotating
        self.wall_token_approximation = wall_token_approximation
        self.move_after_found_wall_token = move_after_found_wall_token

    def execute(
        self,
        robot: Robot,
    ) -> bool:
        from .move import Move
        from .rotate import Rotate

        wall_tokens_by_side: list[tuple[Literal["left", "right"], WallToken | None]] = [
            (
                "left",
                reconhece_lado(
                    robot.camera._left_camera,
                    "left",
                    robot.lidar,
                    rotating=self.rotating,
                ),
            ),
            (
                "right",
                reconhece_lado(
                    robot.camera._right_camera,
                    "right",
                    robot.lidar,
                    rotating=self.rotating,
                ),
            ),
        ]

        found = False

        for side, wall_token in wall_tokens_by_side:
            if not wall_token:
                continue
            robot.motor.stop()
            if self.wall_token_approximation:
                to_move = 0.0
                to_move = robot.lidar.get_side_distance(
                    side, use_min=True, field_of_view=40 * DEGREE_IN_RAD
                )
                robot.run(Rotate(direction=side, turn_angle=PI / 2, just_rotate=True))
                logger.info("go to wall token", System.movement_reason)
                robot.run(
                    Move("forward", to_move / 2, maze=Maze(), just_move=True)
                )  # TODO-: test Maze
            delay(robot.webots_robot, 1300)
            robot.communicator.send_wall_token_information(
                robot.gps.get_position(), wall_token
            )
            if self.wall_token_approximation:
                logger.info("return from wall token", System.movement_reason)
                robot.run(
                    Move("backward", to_move / 2, maze=Maze(), just_move=True)
                )  # TODO-: test Maze
                robot.run(
                    Rotate(
                        direction="right" if side == "left" else "left",
                        turn_angle=PI / 2,
                        just_rotate=True,
                    )
                )
            if self.move_after_found_wall_token:
                robot.motor.set_velocity(6.24 - 3.5, 6.24 - 3.5)
                delay(robot.webots_robot, 100)
                robot.motor.stop()
            found = True
        return found
