import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
from types_and_constants import DEBUG

from .device import Device
from .gps import GPS


class Motor(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        gps: GPS,
        debug_info: DebugInfo,
        left_motor_name: str = "left motor",
        right_motor_name: str = "right motor",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self.debug_info = debug_info

        self._robot = robot
        self._time_step = time_step
        self._left_motor = robot.getDevice(left_motor_name)
        self._right_motor = robot.getDevice(right_motor_name)

        self._left_motor.setPosition(float("inf"))
        self._right_motor.setPosition(float("inf"))
        self.stop()

    def set_left_velocity(self, velocity: float) -> None:
        self._left_motor.setVelocity(velocity)
        if DEBUG:
            self.debug_info.send(
                f"- Definindo velocidade do motor esquerdo: {velocity}",
                System.motor_velocity,
            )

    def set_right_velocity(self, velocity: float) -> None:
        self._right_motor.setVelocity(velocity)
        if DEBUG:
            self.debug_info.send(
                f"- Definindo velocidade do motor direito: {velocity}",
                System.motor_velocity,
            )

    def set_velocity(self, left_velocity: float, right_velocity: float) -> None:
        self.set_left_velocity(left_velocity)
        self.set_right_velocity(right_velocity)

    def stop(self) -> None:
        if DEBUG:
            self.debug_info.send("Parar robô", System.motor_movement)
        self.set_velocity(0, 0)
