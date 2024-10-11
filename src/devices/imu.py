import os

from debugging import DebugInfo, System
from helpers import cyclic_angle
from types_and_constants import DEBUG, PI

from .device import Device


class IMU(Device):
    def __init__(
        self,
        robot,
        debug_info: DebugInfo,
        imu_name: str = "inertial_unit",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._imu = robot.getDevice(imu_name)
        self._imu.enable(time_step)

        self.debug_info = debug_info

        self.start_rotation_angle = None

    def get_rotation_angle(self) -> float:
        # TODO: maybe guarantee that robot is aligned in tile
        rotation_angle = self._imu.getRollPitchYaw()[2]
        if self.start_rotation_angle is None:
            self.start_rotation_angle = rotation_angle
            self.debug_info.send(
                f"Ângulo de rotação inicial do robô, que virará o ângulo 0: {rotation_angle}",
                System.initialization,
            )

        # TODO: check if imu always increase rotating left or it shouldn't be inverted
        # OBS: 2*PI - angle is used because it increases rotating left and other devices
        # decrease in this direction, with this transformation, imu angle is indexed as
        # other devices
        rotation_angle = 2 * PI - cyclic_angle(
            rotation_angle - self.start_rotation_angle
        )
        if DEBUG:
            self.debug_info.send(
                f"Ângulo do robô: {rotation_angle}", System.imu_measures
            )
        return rotation_angle

    @staticmethod
    def get_delta_rotation(ang: float, new_ang: float):
        """
        Get delta between rotation angles from IMU (that ranges from 0 to 2PI).

        WARNING! The angle must have changed just a little, as this
        assumption is used to calculate the delta of the angle. It
        is recommended to the change corresponds to only a time_step rotation
        """
        if abs(new_ang - ang) <= PI:
            return abs(new_ang - ang)
        return min(ang, new_ang) + (2 * PI - max(ang, new_ang))
