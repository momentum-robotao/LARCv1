import os

from controller import Robot as WebotsRobot  # type: ignore

from .device import Device


class Camera(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        camera_name: str = "camera2",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._left_camera = robot.getDevice(camera_name)
        self._left_camera.enable(time_step)
        self._right_camera = self._left_camera
