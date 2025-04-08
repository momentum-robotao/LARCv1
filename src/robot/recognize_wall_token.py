from datetime import datetime

from maze import Maze
from recognize_wall_token import recognize_wall_tokens, verify_wall_tokens
from types_and_constants import TIME_BETWEEN_WT

from .robot import Robot, RobotCommand


class RecognizeWallToken(RobotCommand[None]):
    def __init__(self, maze: Maze):
        self.maze = maze

    def execute(
        self,
        robot: Robot,
    ) -> None:
        time_since_recognition = datetime.now() - robot.last_wt_recognition
        if time_since_recognition < TIME_BETWEEN_WT:
            return
        robot.last_wt_recognition = datetime.now()

        wall_tokens = recognize_wall_tokens(robot.camera)
        wall_tokens_data = verify_wall_tokens(wall_tokens, robot.gps, robot.imu)
        for wall_token in wall_tokens_data:
            self.maze.add_wall_token(wall_token)
