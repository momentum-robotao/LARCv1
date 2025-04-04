from maze import Maze
from recognize_wall_token import recognize_wall_tokens, verify_wall_tokens

from .robot import Robot, RobotCommand


class RecognizeWallToken(RobotCommand[None]):
    def __init__(self, maze: Maze):
        self.maze = maze

    def execute(
        self,
        robot: Robot,
    ) -> None:
        wall_tokens = recognize_wall_tokens(robot.camera)
        wall_tokens_data = verify_wall_tokens(wall_tokens, robot.gps, robot.imu)
        for wall_token in wall_tokens_data:
            self.maze.add_wall_token(wall_token)
