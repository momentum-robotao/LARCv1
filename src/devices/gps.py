import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import RobotLogger, System
from types_and_constants import Coordinate

from .device import Device


class GPS(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        logger: RobotLogger,
        gps_name: str = "gps",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._gps = robot.getDevice(gps_name)
        self._gps.enable(time_step)

        self.logger = logger

    def get_position(self) -> Coordinate:
        positions = self._gps.getValues()
        x, _z, y = positions
        coordinate = Coordinate(x, y)
        self.logger.info(str(coordinate), System.gps_measures)
        return coordinate
