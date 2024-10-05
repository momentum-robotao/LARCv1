import os
from typing import Any

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
from devices import GPS, IMU, ColorSensor, Lidar, Motor
from types_and_constants import DEBUG


class Robot:
    def __init__(
        self,
        webots_robot: WebotsRobot,
        motor: Motor,
        lidar: Lidar,
        gps: GPS,
        imu: IMU,
        color_sensor: ColorSensor,
        debug_info: DebugInfo,
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ):
        self.webots_robot = webots_robot

        self.motor = motor
        self.lidar = lidar
        self.gps = gps
        self.imu = imu
        self.color_sensor = color_sensor

        self.time_step = time_step
        self.debug_info = debug_info

    def show_initialization_information(self) -> None:
        self.debug_info.send(
            "========= Informações do robô inicializado ==========",
            System.initialization,
        )
        if DEBUG:
            self.debug_info.send("Modo do robô: teste", System.initialization)

            self.lidar.show_initialization_information()
        else:
            self.debug_info.send("Modo do robô: competição", System.initialization)
        self.debug_info.send(
            "===================================================\n",
            System.initialization,
        )

    def step(self) -> Any:
        return self.webots_robot.step(self.time_step)
