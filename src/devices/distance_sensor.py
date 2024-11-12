import os
from typing import Literal

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System

from .device import Device

DistanceSensorPosition = Literal["left", "right"]
HolePosition = Literal["left", "right", "central"]


class DistanceSensor(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        debug_info: DebugInfo,
        left_name: str = "ds2",
        right_name: str = "ds1",
        front_name: str = "ds3",
        back_left : str = 'ds4',
        back_right:str = 'ds5',
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self.debug_info = debug_info

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

    def detect_hole(self, webots_robot: WebotsRobot) -> HolePosition | None:
        # left_distances: list[float] = []
        # right_distances: list[float] = []
        # while webots_robot.step(32) != -1 and len(right_distances) < 5:
        #     left_distances.append(self.get_distance("left"))
        #     right_distances.append(self.get_distance("right"))
        if self.get_distance("left") > 0.2 and self.get_distance("right") > 0.2:
            self.debug_info.send("Buraco central", System.hole_detection)
            return "central"

        if self.get_distance("left") > 0.2:
            self.debug_info.send("Buraco esquerda", System.hole_detection)
            return "left"
        elif self.get_distance("right") > 0.2:
            self.debug_info.send("Buraco direita", System.hole_detection)
            return "right"

        self.debug_info.send("Buraco não encontrado", System.hole_detection)
        return None
