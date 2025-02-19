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
        direction: Literal["left", "right"],
        turn_angle: float,
        *,
        correction_rotation: bool = False,
        speed_controller: RotationVelocityController = create_rotation_velocity_controller(),
        just_rotate: bool = False,
        dfs_rotation: bool = True,
    ):
        self.direction = direction
        self.turn_angle = turn_angle
        self.correction_rotation = correction_rotation
        self.speed_controller = speed_controller
        self.just_rotate = just_rotate
        self.dfs_rotation = dfs_rotation

    @log_process(
        [
            "direction",
            "turn_angle",
            "correction_rotation",
            "just_rotate",
            "dfs_rotation",
        ],
        System.rotation,
        from_self=True,
    )
    def execute(self, robot: Robot) -> None:
        """
        Rotate the robot in a direction by an angle, using the motors. Uses
        `imu` to check the robot angle to rotate correctly.
        """
        was_rotating = robot.rotating > 0
        robot.rotating += 1
        rotation_angle = robot.imu.get_rotation_angle()

        logger.info(
            f"     rotacionando {self.turn_angle / DEGREE_IN_RAD} para {self.direction}. "
            f"Era {robot.expected_angle / DEGREE_IN_RAD}",
            System.rotation,
        )
        if not self.correction_rotation and not was_rotating and self.dfs_rotation:
            robot.expected_angle = cyclic_angle(
                robot.expected_angle
                + (-1 if self.direction == "left" else 1) * self.turn_angle
            )
            for test_angle_degree in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
                test_angle = test_angle_degree * DEGREE_IN_RAD
                if abs(test_angle - robot.expected_angle) <= 0.3:
                    robot.expected_angle = test_angle
            if self.direction == "left":  # changed
                if robot.expected_angle > rotation_angle:
                    self.turn_angle = 2 * PI - (robot.expected_angle - rotation_angle)
                else:
                    self.turn_angle = rotation_angle - robot.expected_angle
            else:
                if robot.expected_angle > rotation_angle:
                    self.turn_angle = robot.expected_angle - rotation_angle
                else:
                    self.turn_angle = 2 * PI - (rotation_angle - robot.expected_angle)

        robot.motor.stop()

        angle_accumulated_delta = 0

        while robot.step() != -1:
            new_robot_angle = robot.imu.get_rotation_angle()
            angle_accumulated_delta += IMU.get_delta_rotation(
                rotation_angle, new_robot_angle
            )
            angle_to_rotate = self.turn_angle - angle_accumulated_delta

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

            if angle_accumulated_delta >= self.turn_angle:
                robot.motor.stop()

                logger.info(
                    f"=== Terminou de girar {angle_accumulated_delta / DEGREE_IN_RAD} "
                    f"para {self.direction}. ",
                    System.rotation,
                )
                logger.info(
                    f"Esperado {robot.expected_angle / DEGREE_IN_RAD}. "
                    f"Parou em: {robot.imu.get_rotation_angle() / DEGREE_IN_RAD}",
                    System.rotation_angle_correction,
                )

                logger.info(
                    f"Girou a mais {(angle_accumulated_delta - self.turn_angle)/DEGREE_IN_RAD}",
                    System.rotation_angle_correction,
                )

                if angle_accumulated_delta - self.turn_angle >= 0.1:  # changed
                    robot.run(
                        Rotate(
                            "left" if self.direction == "right" else "right",
                            angle_accumulated_delta - self.turn_angle,
                            correction_rotation=True,
                            speed_controller=create_rotation_velocity_controller(
                                slow_down_angle=angle_accumulated_delta
                                - self.turn_angle
                                + 1
                            ),
                        )
                    )
                    logger.info(
                        f"Girou demais, esperado {robot.expected_angle / DEGREE_IN_RAD}. "
                        f"Parando em: {robot.imu.get_rotation_angle() / DEGREE_IN_RAD}",
                        System.rotation_angle_correction,
                    )

                break
        robot.rotating -= 1


class RotateToAngle(Rotate):
    """
    Rotate the robot to `angle`. It rotates to the direction that
    makes this rotation faster.
    """

    def __init__(
        self,
        angle: float,
        speed_controller: RotationVelocityController = create_rotation_velocity_controller(),
    ):
        self.angle = angle
        self.speed_controller = speed_controller

    def execute(self, robot: Robot) -> None:
        angle_rotating_right = cyclic_angle(2 * PI + self.angle - robot.expected_angle)
        angle_rotating_left = 2 * PI - angle_rotating_right  # complementar angles
        if angle_rotating_right <= PI:
            return robot.run(
                Rotate(
                    "right",
                    angle_rotating_right,
                    speed_controller=self.speed_controller,
                )
            )
        else:
            return robot.run(
                Rotate(
                    "left",
                    angle_rotating_left,
                    speed_controller=self.speed_controller,
                )
            )


rotate_180 = Rotate(direction="right", turn_angle=PI)
