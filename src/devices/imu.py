import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import RobotLogger, System
from helpers import cyclic_angle
from types_and_constants import PI

from .device import Device


class IMU(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        logger: RobotLogger,
        imu_name: str = "inertial_unit",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._imu = robot.getDevice(imu_name)
        self._imu.enable(time_step)

        self.logger = logger

        self.start_rotation_angle = None

    def get_rotation_angle(self, raw: bool = False) -> float:
        rotation_angle = self._imu.getRollPitchYaw()[2]
        if self.start_rotation_angle is None:
            self.start_rotation_angle = rotation_angle
            self.logger.info(
                f"Ângulo de rotação inicial do robô, que virará o ângulo 0: {rotation_angle}",
                System.initialization,
            )
        if raw:
            return rotation_angle - self.start_rotation_angle

        # ? 2*PI - angle is used because it increases rotating left and other devices
        # decrease in this direction, with this transformation, imu angle is indexed as
        # other devices
        rotation_angle = cyclic_angle(
            2 * PI - (rotation_angle - self.start_rotation_angle)
        )
        self.logger.info(f"Ângulo do robô: {rotation_angle}", System.imu_measures)
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

    # TODO-: unify with get_rotation_angle
    def get_roll_pitch_yaw(self) -> float:
        return self._imu.getRollPitchYaw()[2]
