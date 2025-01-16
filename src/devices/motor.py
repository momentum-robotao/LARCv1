import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import System, logger

from .device import Device


class Motor(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        left_motor_name: str = "left motor",
        right_motor_name: str = "right motor",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._robot = robot
        self._time_step = time_step
        self._left_motor = robot.getDevice(left_motor_name)
        self._right_motor = robot.getDevice(right_motor_name)

        self._left_motor.setPosition(float("inf"))
        self._right_motor.setPosition(float("inf"))
        self.stop()

    def set_left_velocity(self, velocity: float) -> None:
        self._left_motor.setVelocity(velocity)
        logger.info(
            f"- Definindo velocidade do motor esquerdo: {velocity}",
            System.motor_velocity,
        )

    def set_right_velocity(self, velocity: float) -> None:
        self._right_motor.setVelocity(velocity)
        logger.info(
            f"- Definindo velocidade do motor direito: {velocity}",
            System.motor_velocity,
        )

    def set_velocity(self, left_velocity: float, right_velocity: float) -> None:
        self.set_left_velocity(left_velocity)
        self.set_right_velocity(right_velocity)

    def stop(self) -> None:
        logger.info("Parar robô", System.motor_movement)
        self.set_velocity(0, 0)
