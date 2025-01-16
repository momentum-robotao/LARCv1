import os

from controller import Robot as WebotsRobot  # type: ignore

from .device import Device


class Camera(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        left_camera_name: str = "cameraE",
        right_camera_name: str = "cameraD",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._left_camera = robot.getDevice(left_camera_name)
        self._left_camera.enable(time_step)
        self._right_camera = robot.getDevice(right_camera_name)
        self._right_camera.enable(time_step)
