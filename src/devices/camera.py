import os

import numpy as np
from controller import Robot as WebotsRobot  # type: ignore
from numpy.typing import NDArray

from .device import Device


class Camera(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        camera_name: str = "camera2",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._camera = robot.getDevice(camera_name)
        self._camera.enable(time_step)

        self.width = self._camera.getWidth()
        self.height = self._camera.getHeight()

        self.FOV = 0.6

    def get_image(self) -> bytes:
        return self._camera.getImage()

    def get_rgb_matrix(self) -> NDArray[np.uint8]:
        camera_image = self.get_image()

        image_argb = np.frombuffer(camera_image, dtype=np.uint8).reshape(
            (self.height, self.width, 4)
        )

        image = np.copy(image_argb[:, :, :3])

        return image
