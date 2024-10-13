import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
from types_and_constants import DEBUG, Coordinate

from .device import Device


class GPS(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        debug_info: DebugInfo,
        gps_name: str = "gps",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._gps = robot.getDevice(gps_name)
        self._gps.enable(time_step)

        self.debug_info = debug_info

    def get_coordinates(self) -> Coordinate:
        positions = self._gps.getValues()
        x, _z, y = positions
        coordinate = Coordinate(x, y)
        if DEBUG:
            self.debug_info.send(str(coordinate), System.gps_measures)
        return coordinate
