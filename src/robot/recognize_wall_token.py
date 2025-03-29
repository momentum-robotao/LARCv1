from recognize_wall_token import classify_wall_token
from utils import delay

from .robot import Robot, RobotCommand


class RecognizeWallToken(RobotCommand[bool]):
    def execute(
        self,
        robot: Robot,
    ) -> bool:
        wall_token = classify_wall_token(robot.camera, robot.gps, robot.imu)
        if not wall_token:
            return False

        robot.motor.stop()
        delay(robot.webots_robot, 1300)
        robot.communicator.send_wall_token_information(
            robot.gps.get_position(), wall_token
        )

        return True
