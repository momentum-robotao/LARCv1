from typing import Literal

from debugging import System, log_process, logger
from devices import IMU
from types_and_constants import DEGREE_IN_RAD, PI
from utils import cyclic_angle

from .robot import Robot, RobotCommand
from .velocity_controller import (
    RotationVelocityController,
    create_rotation_velocity_controller,
)


class Rotate(RobotCommand):
    def __init__(
        self,
        direction: Literal["left", "right", "fastest"],
        angle: float,
        *,
        correction_rotation: bool = False,
        speed_controller: RotationVelocityController = create_rotation_velocity_controller(),
    ):
        """
        Rotate the robot in a direction by an angle, using the motors. Uses
        `imu` to check the robot angle to rotate correctly.

        :param direction: If `fastest`, angle is the desired final angle and not the angle to turn.
        """
        self.direction = direction
        self.angle = angle
        self.correction_rotation = correction_rotation
        self.speed_controller = speed_controller

    @log_process(
        [
            "direction",
            "angle",
            "correction_rotation",
        ],
        System.rotation,
        from_self=True,
    )
    def execute(self, robot: Robot) -> None:
        rotation_angle = robot.imu.get_rotation_angle()

        logger.info(
            f"     rotacionando {self.angle / DEGREE_IN_RAD} para {self.direction}. "
            f"Era {robot.expected_angle / DEGREE_IN_RAD}",
            System.rotation,
        )

        new_expected_angle = (
            cyclic_angle(
                robot.expected_angle
                + (-1 if self.direction == "left" else 1) * self.angle
            )
            if self.direction != "fastest"
            else self.angle
        )
        if not self.correction_rotation:
            for test_angle_degree in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
                test_angle = test_angle_degree * DEGREE_IN_RAD
                if abs(test_angle - robot.expected_angle) <= 10:
                    new_expected_angle = test_angle
            if self.direction == "left":  # changed
                if new_expected_angle > rotation_angle:
                    self.angle = 2 * PI - (new_expected_angle - rotation_angle)
                else:
                    self.angle = rotation_angle - new_expected_angle
            elif self.direction == "right":
                if new_expected_angle > rotation_angle:
                    self.angle = new_expected_angle - rotation_angle
                else:
                    self.angle = 2 * PI - (rotation_angle - new_expected_angle)

        if self.direction == "fastest":
            if new_expected_angle > rotation_angle:
                if new_expected_angle - rotation_angle < PI:
                    self.angle = new_expected_angle - rotation_angle
                    self.direction = "right"
                else:
                    self.angle = 2 * PI - (new_expected_angle - rotation_angle)
                    self.direction = "left"
            else:
                if rotation_angle - new_expected_angle < PI:
                    self.angle = rotation_angle - new_expected_angle
                    self.direction = "left"
                else:
                    self.angle = 2 * PI - (rotation_angle - new_expected_angle)
                    self.direction = "right"

        robot.motor.stop()

        angle_accumulated_delta = 0

        while robot.step() != -1:
            new_robot_angle = robot.imu.get_rotation_angle()
            angle_accumulated_delta += IMU.get_delta_rotation(
                rotation_angle, new_robot_angle
            )
            angle_to_rotate = self.angle - angle_accumulated_delta

            logger.info(
                f"- Já girou {angle_accumulated_delta / DEGREE_IN_RAD} no total, falta "
                f"{angle_to_rotate / DEGREE_IN_RAD}",
                System.rotation_step_by_step,
            )

            rotation_angle = new_robot_angle

            left_velocity, right_velocity = self.speed_controller(
                angle_to_rotate, self.direction
            )
            robot.motor.set_velocity(left_velocity, right_velocity)

            if angle_accumulated_delta >= self.angle:
                robot.motor.stop()

                if not self.correction_rotation:
                    robot.expected_angle = new_expected_angle

                logger.info(
                    f"=== Terminou de girar {angle_accumulated_delta / DEGREE_IN_RAD} "
                    f"para {self.direction}. ",
                    System.rotation,
                )
                logger.info(
                    f"Esperado {robot.expected_angle / DEGREE_IN_RAD}. "
                    f"Parou em: {robot.imu.get_rotation_angle() / DEGREE_IN_RAD}.\n"
                    f"Girou a mais {(angle_accumulated_delta - self.angle)/DEGREE_IN_RAD}",
                    System.rotation_angle_correction,
                )

                if angle_accumulated_delta - self.angle >= 0.1:  # changed
                    robot.run(
                        Rotate(
                            "left" if self.direction == "right" else "right",
                            angle_accumulated_delta - self.angle,
                            correction_rotation=True,
                            speed_controller=create_rotation_velocity_controller(
                                slow_down_angle=angle_accumulated_delta - self.angle + 1
                            ),
                        )
                    )
                    logger.info(
                        f"Girou demais, esperado {robot.expected_angle / DEGREE_IN_RAD}. "
                        f"Parando em: {robot.imu.get_rotation_angle() / DEGREE_IN_RAD}",
                        System.rotation_angle_correction,
                    )

                break


rotate_180 = Rotate(direction="right", angle=PI)
