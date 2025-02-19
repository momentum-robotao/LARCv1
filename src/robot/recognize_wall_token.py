from typing import Literal

from recognize_wall_token import reconhece_lado
from types_and_constants import WallToken
from utils import delay

from .robot import Robot, RobotCommand


class RecognizeWallToken(RobotCommand[bool]):
    def execute(
        self,
        robot: Robot,
    ) -> bool:
        wall_tokens_by_side: list[tuple[Literal["left", "right"], WallToken | None]] = [
            (
                "left",
                reconhece_lado(
                    robot.camera._left_camera,
                    "left",
                    robot.lidar,
                ),
            ),
            (
                "right",
                reconhece_lado(
                    robot.camera._right_camera,
                    "right",
                    robot.lidar,
                ),
            ),
        ]

        found = False

        for side, wall_token in wall_tokens_by_side:
            if not wall_token:
                continue
            robot.motor.stop()
            delay(robot.webots_robot, 1300)
            robot.communicator.send_wall_token_information(
                robot.gps.get_position(), wall_token
            )
            found = True
        return found
