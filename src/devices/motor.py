import os
from typing import Literal

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
from helpers import cyclic_angle, round_if_almost_0
from types_and_constants import (
    DEBUG,
    EXPECTED_WALL_DISTANCE,
    KP,
    MAX_SPEED,
    PI,
    SLOW_DOWN_DIST,
    SLOW_DOWN_SPEED,
    TILE_SIZE,
    MovementResult,
    WallColisionError,
)

# TODO: this devices should not be accessed by other devices, this indicates that these
# functions should be in `Robot` from `robot.py`
from .color_sensor import ColorSensor
from .device import Device
from .gps import GPS
from .imu import IMU
from .lidar import Lidar


class Motor(Device):
    def __init__(
        self,
        robot: WebotsRobot,
        debug_info: DebugInfo,
        left_motor_name: str = "left motor",
        right_motor_name: str = "right motor",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self.debug_info = debug_info

        self._robot = robot
        self._time_step = time_step
        self._left_motor = robot.getDevice(left_motor_name)
        self._right_motor = robot.getDevice(right_motor_name)

        self._left_motor.setPosition(float("inf"))
        self._right_motor.setPosition(float("inf"))
        self.stop()

        self._angle_imprecision = 0.0
        self._move_imprecision = 0.0

    def _get_angle_imprecision(self, direction: Literal["left", "right"]) -> float:
        return self._angle_imprecision * (-1 if direction == "left" else 1)

    def _set_angle_imprecision(
        self, value: float, direction: Literal["left", "right"]
    ) -> None:
        self._angle_imprecision = value * (-1 if direction == "left" else 1)

    def _get_move_imprecision(self, direction: Literal["forward", "backward"]) -> float:
        return self._move_imprecision * (-1 if direction == "backward" else 1)

    def _set_move_imprecision(
        self, value: float, direction: Literal["forward", "backward"]
    ) -> None:
        self._move_imprecision = value * (-1 if direction == "backward" else 1)

    def stop(self) -> None:
        if DEBUG:
            self.debug_info.send("Parar robô", System.motor_movement)
        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)

    def rotate(
        self,
        direction: Literal["left", "right"],
        turn_angle: float,
        imu: IMU,
        slow_down_angle: float = 0.1,
        high_speed: float = MAX_SPEED,
        low_speed: float = MAX_SPEED / 100,
    ) -> None:
        """
        Rotate the robot in a direction by an angle, using the motors. Uses
        `imu` to check the robot angle to rotate correctly.
        """

        self.stop()

        if DEBUG:
            self.debug_info.send(
                f"=== Começando a girar {turn_angle} rad para {direction}, "
                f"com a correção de {self._get_angle_imprecision(direction)} "
                "rad ===",
                System.motor_rotation,
            )

        turn_angle -= self._get_angle_imprecision(direction)
        self._set_angle_imprecision(0, direction)

        if turn_angle <= 0.00001:
            return

        angle_accumulated_delta = 0
        rotation_angle = imu.get_rotation_angle()

        while self._robot.step(self._time_step) != -1:
            new_robot_angle = imu.get_rotation_angle()
            angle_accumulated_delta += IMU.get_delta_rotation(
                rotation_angle, new_robot_angle
            )
            angle_to_rotate = turn_angle - angle_accumulated_delta

            if DEBUG:
                self.debug_info.send(
                    f"- Já girou {angle_accumulated_delta} no total, falta "
                    f"{angle_to_rotate}",
                    System.motor_rotation,
                )

            rotation_angle = new_robot_angle

            left_velocity, right_velocity = Motor.rotation_velocity_controller(
                angle_to_rotate, direction, slow_down_angle, high_speed, low_speed
            )
            self._left_motor.setVelocity(left_velocity)
            self._right_motor.setVelocity(right_velocity)
            if DEBUG:
                self.debug_info.send(
                    "- Definindo velocidade dos motores na rotação: "
                    f"esquerdo={left_velocity} e direito={right_velocity}",
                    System.motor_velocity,
                )

            if angle_accumulated_delta >= turn_angle:
                self._set_angle_imprecision(
                    angle_accumulated_delta - turn_angle, direction
                )

                self.stop()

                if DEBUG:
                    self.debug_info.send(
                        f"=== Terminou de girar {angle_accumulated_delta} "
                        f"para {direction}. "
                        f"Passou em {self._get_angle_imprecision(direction)} "
                        "como imprecisão a ser corrigida depois ===",
                        System.motor_rotation,
                    )

                break

    def rotate_180(self, imu: IMU) -> None:
        """Rotate 180 degrees."""
        self.rotate("right", PI, imu)

    def rotate_90_left(self, imu: IMU) -> None:
        """Rotate 90 degrees to left."""
        self.rotate("left", PI / 2, imu)

    def rotate_90_right(self, imu: IMU) -> None:
        """Rotate 90 degrees to right."""
        self.rotate("right", PI / 2, imu)

    def rotate_to_angle(
        self,
        angle: float,
        imu: IMU,
        slow_down_angle: float = 0.1,
        high_speed: float = MAX_SPEED,
        low_speed: float = MAX_SPEED / 100,
    ) -> None:
        """
        Rotate the robot to `angle`. It rotates to the direction that
        makes this rotation faster.
        """
        robot_rotation_angle = imu.get_rotation_angle()
        angle_rotating_right = cyclic_angle(2 * PI + angle - robot_rotation_angle)
        angle_rotating_left = 2 * PI - angle_rotating_right  # complementar angles
        if angle_rotating_right <= PI:
            self.rotate(
                "right",
                angle_rotating_right,
                imu,
                slow_down_angle,
                high_speed,
                low_speed,
            )
        else:
            self.rotate(
                "left",
                angle_rotating_left,
                imu,
                slow_down_angle,
                high_speed,
                low_speed,
            )

    def move(
        self,
        direction: Literal["forward", "backward"],
        gps: GPS,
        lidar: Lidar,
        color_sensor: ColorSensor,
        dist: float = TILE_SIZE,
        slow_down_dist: float = SLOW_DOWN_DIST,
        high_speed: float = MAX_SPEED,
        slow_down_speed: float = SLOW_DOWN_SPEED,
        kp: float = KP,
        expected_wall_distance: float = EXPECTED_WALL_DISTANCE,
        returning_to_safe_position: bool = False,
    ) -> MovementResult:
        # TODO: diferenciar hole/parede de quando tá indo pra trás e para frente
        """
        Move the robot by certain distance in meters in some direction, using
        the motors. If it enconters a hole, it raises `` and if it collide
        with a wall, it raises `WallColisionError`, but in both cases, the
        robot come back to the initial position before.

        :param direction: Direction that robot should move.
        :param gps: Used to check the robot position to navigate correctly.
        :param lidar: Used to check wall distances and adjust the robot
                      rotation angle.
        :param color_sensor: Used to recognize holes.
        :param dist: Distance that the robot should try to move.
        :param kp: Intensity of robot rotation angle corrections.
        :param expected_wall_distance: The distance from the wall that the
                                       robot is going to try to maintain.
        :param returning_to_safe_position: If robot is returning to a safe
            position is expected that it will recognize a hole or wall while
            returning to last position and it is guaranteeded that the movement
            is safe/allowed, so it doesn't stop with holes nor walls.
        :return: What was the result of moving. For example, if it has moved or
            there were a hole and it returned to its start position.
        :raises WallColisionError: If the robot collides with a wall for some
            reason, it will return to its position before this movement and
            then raise this exception.

        OBS:
        - Moves at `high_speed`, until it lasts `slow_down_dist` to move, when
        it slows down to `slow_down_speed`. This approach is intended to make
        the movement quick in general, but with a good precision as it moves
        slower in the end of the movement.
        - Holes are not detected when moving backward.  TODO: solve this?
        """
        initial_position = gps.get_coordinates()

        if DEBUG:
            self.debug_info.send(
                f"Começando a mover {dist} para {direction}, com imprecisão "
                f"{self._get_move_imprecision(direction)} a ser corrigida.",
                System.motor_movement,
            )

        dist -= self._get_move_imprecision(direction)
        self._set_move_imprecision(0, direction)

        while self._robot.step(self._time_step) != -1:
            actual_position = gps.get_coordinates()

            rotation_angle_error = lidar.get_rotation_angle_error(
                expected_wall_distance, kp
            )

            x_delta = round_if_almost_0(abs(actual_position.x - initial_position.x))
            y_delta = round_if_almost_0(abs(actual_position.y - initial_position.y))
            traversed_dist = x_delta + y_delta

            if DEBUG:
                self.debug_info.send(
                    f"Já moveu {traversed_dist}, sendo {x_delta=} e {y_delta=}.",
                    System.motor_movement,
                )

            left_velocity, right_velocity = self.movement_velocity_controller(
                dist - traversed_dist,
                direction,
                rotation_angle_error,
                slow_down_dist,
                high_speed,
                slow_down_speed,
            )
            self._left_motor.setVelocity(left_velocity)
            self._right_motor.setVelocity(right_velocity)

            if DEBUG:
                self.debug_info.send(
                    f"Definindo velocidades como: esquerda={left_velocity} e "
                    f"direita={right_velocity}",
                    System.motor_velocity,
                )

            if traversed_dist >= dist:
                self.stop()

                self._set_move_imprecision(traversed_dist - dist, direction)

                if DEBUG:
                    self.debug_info.send(
                        f"Fim do movimento, andou {traversed_dist} do "
                        f"objetivo: {dist} para {direction}. Passou em "
                        f"{self._get_move_imprecision(direction)} como "
                        "imprecisão para ser corrigida depois.",
                        System.motor_movement,
                    )

                break

            if (
                lidar.wall_collision("front" if direction == "forward" else "back")
                and not returning_to_safe_position
            ):
                self.stop()
                self.move(
                    "backward" if direction == "forward" else "forward",
                    gps,
                    lidar,
                    color_sensor,
                    traversed_dist,
                    slow_down_dist,
                    high_speed,
                    slow_down_speed,
                    kp,
                    expected_wall_distance,
                    returning_to_safe_position=True,
                )

                if DEBUG:
                    self.debug_info.send(
                        "Retornou à posição antiga após colidir com parede.",
                        System.lidar_wall_detection,
                    )

                raise WallColisionError()

            # TODO: refator - consider: create deal_with_hole(); data structure for
            # movement information, ex: slow_down_dist, high_speed, slow_speed...
            if color_sensor.has_hole() and not returning_to_safe_position:
                self.stop()

                # TODO: verificar se buraco é no quarter tile da esquerda e/ou direita
                # self.rotate(
                #     "left",
                #     20 * DEGREE_IN_RAD,
                #     imu,
                #     slow_down_angle,
                #     high_speed,
                #     low_speed,
                # )

                # TODO: check hole when moving backward, doesn't sensor is only on front side?
                self.move(
                    "backward" if direction == "forward" else "forward",
                    gps,
                    lidar,
                    color_sensor,
                    traversed_dist,
                    slow_down_dist,
                    high_speed,
                    slow_down_speed,
                    kp,
                    expected_wall_distance,
                    returning_to_safe_position=True,
                )

                if DEBUG:
                    self.debug_info.send(
                        "Retornou à posição antiga após achar buraco.",
                        System.lidar_wall_detection,
                    )

                return MovementResult.left_right_hole
        return MovementResult.moved

    @staticmethod
    def rotation_velocity_controller(
        angle_to_rotate: float,
        direction: Literal["left", "right"],
        slow_down_angle: float = 0.1,
        high_speed: float = MAX_SPEED,
        low_speed: float = MAX_SPEED / 100,
    ) -> tuple[float, float]:
        """
        It returns the `high_speed` if the robot has already to rotate a lot
        (more than `slow_down_angle`), else it returns the `low_speed`.

        :param angle_to_rotate: The angle that the robot haven't rotated yet
                                and is going to rotate.

        :return: A tuple with (left velocity, right velocity) according to the
        angle it didn't rotate yet, the direction that each motor should rotate
        is indicated by the signal of these values.
        """
        speed = high_speed if angle_to_rotate > slow_down_angle else low_speed
        if direction == "left":
            left_velocity = -1 * speed
            right_velocity = speed
        elif direction == "right":
            left_velocity = speed
            right_velocity = -1 * speed
        return left_velocity, right_velocity

    def movement_velocity_controller(
        self,
        dist_to_move: float,
        direction: Literal["forward", "backward"],
        rotation_angle_error: float,
        slow_down_dist: float = 0.001,
        high_speed: float = MAX_SPEED,
        low_speed: float = MAX_SPEED / 100,
    ) -> tuple[float, float]:
        """
        It returns the `high_speed` if the robot has already to move a lot
        (more than `slow_down_dist`), else it returns the `low_speed`.

        :param dist_to_move: The distance that the robot haven't moved yet
                             and is going to move.

        :return: A tuple with (left velocity, right velocity) to the motor
        according to the distance it haven't moved yet and considering the
        `direction` to decide the direction that motors should turn.
        """
        # Recalculate `high_speed` so it is possible to maintain a motor
        # `rotation_angle_error` faster than the other to correct it
        high_speed = min(high_speed, MAX_SPEED - abs(rotation_angle_error))

        if DEBUG and abs(rotation_angle_error) > MAX_SPEED:
            self.debug_info.send(
                "No PID, o erro para ser corrigido da rotação do "
                f"robô é: {rotation_angle_error}, maior do que a "
                f"maior velocidade do robô: {MAX_SPEED}",
                System.motor_velocity,
                "warning",
            )

        speed = high_speed if dist_to_move > slow_down_dist else low_speed
        left_velocity = speed - rotation_angle_error
        right_velocity = speed + rotation_angle_error

        if direction == "backward":
            left_velocity *= -1
            right_velocity *= -1

        return left_velocity, right_velocity
