from typing import Literal, Protocol

from types_and_constants import (
    MAX_SPEED,
    SLOW_DOWN_ANGLE,
    SLOW_DOWN_DIST,
    SLOW_DOWN_MOVE_SPEED,
    SLOW_DOWN_ROTATE_SPEED,
    TILE_SIZE,
)


class RotationVelocityController(Protocol):
    def __call__(
        self, remaining_angle: float, direction: Literal["left", "right"]
    ) -> tuple[float, float]:
        """
        Speed is `high_speed` if the robot has to rotate a lot yet
        (more than `slow_down_angle`), else it is `slow_down_speed`.

        :param remaining_angle: The angle that the robot hasn't rotated yet.
        :param direction: In which direction the robot is rotating.

        :return: A tuple `(left velocity, right velocity)`.
        The signal of those values is defined by `direction`.
        """
        ...


def create_rotation_velocity_controller(
    slow_down_angle: float = SLOW_DOWN_ANGLE,
    high_speed: float = MAX_SPEED,
    slow_down_speed: float = SLOW_DOWN_ROTATE_SPEED,
) -> RotationVelocityController:
    def rotation_velocity_controller(
        remaining_angle: float, direction: Literal["left", "right"]
    ) -> tuple[float, float]:
        speed = high_speed if remaining_angle > slow_down_angle else slow_down_speed
        if direction == "left":
            left_velocity = -1 * speed
            right_velocity = speed
        elif direction == "right":
            left_velocity = speed
            right_velocity = -1 * speed
        return left_velocity, right_velocity

    return rotation_velocity_controller


class MovementVelocityController(Protocol):
    def __call__(
        self,
        remaining_distance: float,
        wall_distance: float,
        direction: Literal["forward", "backward"],
    ) -> tuple[float, float]:
        """
        Speed is `high_speed` if the robot has to move a lot yet
        (more than `slow_down_dist`), else it is `slow_down_speed`.

        :param remaining_distance: The distance that the robot hasn't moved yet.
        :param direction: In which direction the robot is moving.

        :return: A tuple `(left velocity, right velocity)`.
        The signal of those values is defined by `direction`.

        This approach is intended to make
        the movement quick in general, but with a good precision as it moves
        slower in the end of the movement.
        """
        ...


def create_movement_velocity_controller(
    slow_down_dist: float = SLOW_DOWN_DIST,
    high_speed: float = MAX_SPEED,
    slow_down_speed: float = SLOW_DOWN_MOVE_SPEED,
    wall_distance_if_wall: float = TILE_SIZE,
    slow_down_dist_if_wall: float = TILE_SIZE / 6,
    slow_down_speed_if_wall: float = MAX_SPEED / 2,
) -> MovementVelocityController:
    def movement_velocity_controller(
        remaining_distance: float,
        wall_distance: float,
        direction: Literal["forward", "backward"],
    ) -> tuple[float, float]:
        speed = high_speed
        if remaining_distance <= slow_down_dist:
            speed = slow_down_speed
        elif (
            wall_distance <= wall_distance_if_wall
            and remaining_distance <= slow_down_dist_if_wall
        ):
            speed = slow_down_speed_if_wall

        left_velocity = speed
        right_velocity = speed

        if direction == "backward":
            left_velocity *= -1
            right_velocity *= -1

        return left_velocity, right_velocity

    return movement_velocity_controller
