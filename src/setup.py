import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import RobotLogger, System
from devices import (
    GPS,
    IMU,
    Camera,
    ColorSensor,
    Communicator,
    DistanceSensor,
    Lidar,
    Motor,
)
from maze import Maze
from robot import Robot, set_dist_change_mapper
from types_and_constants import DEBUG, DEGREE_IN_RAD


def setup_robot(logger: RobotLogger) -> Robot:
    # Initialize robot and devices
    try:
        webots_robot = WebotsRobot()
        webots_robot.step(int(os.getenv("TIME_STEP", 32)))
        lidar = Lidar(webots_robot, logger)
        gps = GPS(webots_robot, logger)
        motor = Motor(webots_robot, logger)
        imu = IMU(webots_robot, logger)
        color_sensor = ColorSensor(webots_robot, logger)
        communicator = Communicator(webots_robot, logger)
        camera = Camera(webots_robot, logger)
        distance_sensor = DistanceSensor(webots_robot, logger)
    except Exception:
        if DEBUG:
            logger.error(
                "Erro durante inicialização dos devices", System.initialization
            )
            raise

    try:
        robot = Robot(
            webots_robot,
            motor,
            lidar,
            gps,
            imu,
            color_sensor,
            communicator,
            camera,
            distance_sensor,
            logger=logger,
        )
        robot.step()
    except Exception:
        if DEBUG:
            logger.error("Erro durante inicialização do robô", System.initialization)
            raise
    return robot


def setup_delta_coordinate_mapper(robot: Robot, maze: Maze):
    robot.imu.get_rotation_angle()
    initial_position = robot.gps.get_position()
    robot.move(
        "forward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    delta_0_cord = robot.gps.get_position() - initial_position
    delta_0 = (delta_0_cord.x, delta_0_cord.y)
    robot.move(
        "backward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    robot.rotate("right", 45 * DEGREE_IN_RAD, correction_rotation=True)
    initial_position = robot.gps.get_position()
    robot.move(
        "forward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    delta_45_cord = robot.gps.get_position() - initial_position
    delta_45 = (delta_45_cord.x, delta_45_cord.y)
    robot.move(
        "backward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    robot.rotate("left", 45 * DEGREE_IN_RAD, correction_rotation=True)
    set_dist_change_mapper(delta_0, delta_45)
