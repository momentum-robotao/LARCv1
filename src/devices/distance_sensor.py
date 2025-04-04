import os
from typing import Literal

from controller import Robot as WebotsRobot  # type: ignore

from debugging import System, logger

from .device import Device

DistanceSensorPosition = Literal["left", "right"]
HolePosition = Literal["left", "right", "central"]


class DistanceSensor(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        left_name: str = "ds2",
        right_name: str = "ds1",
        front_name: str = "ds3",
        back_left: str = "ds4",
        back_right: str = "ds5",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._left = robot.getDevice(left_name)
        self._left.enable(time_step)
        self._right = robot.getDevice(right_name)
        self._right.enable(time_step)
        self._front = robot.getDevice(front_name)
        self._front.enable(time_step)
        self._backleft = robot.getDevice(back_left)
        self._backleft.enable(time_step)
        self._backright = robot.getDevice(back_right)
        self._backright.enable(time_step)

    def get_distance(self, position: DistanceSensorPosition) -> float:
        sensor = getattr(self, f"_{position}")
        return float(sensor.getValue())

    def pegar_distancia(self, sensor):
        return float(sensor.getValue())

    def _check_hole_position(self) -> HolePosition | None:
        if self.get_distance("left") > 0.2 and self.get_distance("right") > 0.2:
            logger.info("Buraco central", System.hole_detection)
            return "central"

        if self.get_distance("left") > 0.2:
            logger.info("Buraco esquerda", System.hole_detection)
            return "left"
        elif self.get_distance("right") > 0.2:
            logger.info("Buraco direita", System.hole_detection)
            return "right"

        return None

    def detect_hole(self) -> HolePosition | None:
        hole = self._check_hole_position()
        logger.info(
            f"{'Não encontrou' if hole is None else 'Encontrou'} buraco no caminho: {hole}",
            System.hole_detection,
        )

        return hole
