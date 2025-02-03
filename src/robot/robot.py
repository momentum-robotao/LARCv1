import os
import time
from abc import ABC, abstractmethod
from typing import Any, Generic, TypeVar

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
from types_and_constants import (
    DEBUG,
    DEGREE_IN_RAD,
    POSSIBLE_ANGLES,
    EndOfTimeError,
    LackOfProgressError,
)

T = TypeVar("T")
V = TypeVar("V")


def round_angle(angle: float) -> int:
    angle_degree = angle / DEGREE_IN_RAD
    most_similar = (0, angle_degree)
    for test_angle in POSSIBLE_ANGLES:
        if most_similar[1] > abs(angle_degree - test_angle):
            most_similar = (test_angle, abs(angle_degree - test_angle))
    return most_similar[0]


class RobotCommand(ABC, Generic[T]):
    @abstractmethod
    def execute(self, robot: "Robot") -> T: ...


class Robot:
    def __init__(
        self,
        webots_robot: WebotsRobot,
        motor: Motor,
        lidar: Lidar,
        gps: GPS,
        imu: IMU,
        color_sensor: ColorSensor,
        communicator: Communicator,
        camera: Camera,
        distance_sensor: DistanceSensor,
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ):
        self.last_check_time_ms = round(time.time() * 1000)
        self.webots_robot = webots_robot

        self.motor = motor
        self.lidar = lidar
        self.gps = gps
        self.imu = imu
        self.color_sensor = color_sensor
        self.communicator = communicator
        self.camera = camera
        self.distance_sensor = distance_sensor

        self.time_step = time_step

        self.expected_angle = 0.0
        self.step()
        self.expected_position = gps.get_position()
        self.rotating = 0
        # ? 5 primeiros minutos não checa tempo. Depois a cada 2 segs
        self.last_check_time_ms = round(time.time() * 1000) + 5 * 60 * 1000

    def check_time(
        self, time_tolerance: int = int(os.getenv("TIME_TOLERANCE", 3))
    ) -> None:
        game_information = self.communicator.get_game_information()
        if (
            game_information.remaining_real_world_time < time_tolerance
            or game_information.remaining_simulation_time < time_tolerance
        ):
            raise EndOfTimeError()
        self.last_check_time_ms = round(time.time() * 1000)

    def show_initialization_information(self) -> None:
        logger.info(
            "========= Informações do robô inicializado ==========",
            System.initialization,
        )
        if DEBUG:
            logger.info("Modo do robô: teste", System.initialization)

            self.lidar.show_initialization_information()
        else:
            print("Modo do robô: competição")
        logger.info(
            "===================================================\n",
            System.initialization,
        )

    def step(self) -> Any:
        if self.communicator.occured_lack_of_progress():
            raise LackOfProgressError()
        actual_time_ms = round(time.time() * 1000)
        if actual_time_ms - self.last_check_time_ms >= 1000:
            self.check_time()
        return self.webots_robot.step(self.time_step)

    def run(self, command: RobotCommand[V]) -> V:
        return command.execute(self)
