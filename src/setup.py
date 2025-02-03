import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import System, logger
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
from robot import Move, Robot, Rotate, set_dist_change_mapper
from types_and_constants import DEBUG, DEGREE_IN_RAD


def setup_robot() -> Robot:
    # Initialize robot and devices
    try:
        webots_robot = WebotsRobot()
        webots_robot.step(int(os.getenv("TIME_STEP", 32)))
        lidar = Lidar(webots_robot)
        gps = GPS(webots_robot)
        motor = Motor(webots_robot)
        imu = IMU(webots_robot)
        color_sensor = ColorSensor(webots_robot)
        communicator = Communicator(webots_robot)
        camera = Camera(webots_robot)
        distance_sensor = DistanceSensor(webots_robot)
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
    robot.run(Move("forward", 0.01, maze=maze, correction_move=True))
    delta_0_cord = robot.gps.get_position() - initial_position
    delta_0 = (delta_0_cord.x, delta_0_cord.y)
    robot.run(Move("backward", 0.01, maze=maze, correction_move=True))
    robot.run(Rotate("right", 45 * DEGREE_IN_RAD, correction_rotation=True))
    initial_position = robot.gps.get_position()
    robot.run(Move("forward", 0.01, maze=maze, correction_move=True))
    delta_45_cord = robot.gps.get_position() - initial_position
    delta_45 = (delta_45_cord.x, delta_45_cord.y)
    robot.run(Move("backward", 0.01, maze=maze, correction_move=True))
    robot.run(Rotate("left", 45 * DEGREE_IN_RAD, correction_rotation=True))
    set_dist_change_mapper(delta_0, delta_45)
