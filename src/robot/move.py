from enum import Enum
from typing import Literal

from debugging import System, log_process, logger
from maze import Maze
from types_and_constants import (
    DEGREE_IN_RAD,
    DIST_BEFORE_HOLE,
    PI,
    POSSIBLE_ANGLES,
    Coordinate,
)
from utils import cyclic_angle, round_if_almost_0

from .robot import Robot, RobotCommand
from .velocity_controller import (
    MovementVelocityController,
    create_movement_velocity_controller,
)


class MovementResult(Enum):
    moved = "moved"
    left_hole = "left hole"
    right_hole = "right hole"
    central_hole = "central hole"
    unavoidable_obstacle = "unavoidable obstacle"


DIST_CHANGE_MAPPER: dict[int, tuple[float, float]] = {}

SQRT_2 = 1.414213562373

ordered_deltas: list[tuple[float, float]] = [
    (0, 1),  # 0°
    (-1.0 / SQRT_2, 1.0 / SQRT_2),  # 45°
    (-1, 0),  # 90°
    (-1.0 / SQRT_2, -1.0 / SQRT_2),
    (0, -1),
    (1.0 / SQRT_2, -1.0 / SQRT_2),
    (1, 0),
    (1.0 / SQRT_2, 1.0 / SQRT_2),
]


def get_signal(val: float) -> int:
    if round_if_almost_0(val) == 0:
        return 0
    return 1 if val > 0 else -1


def get_signal_delta(delta: tuple[float, float]) -> tuple[int, int]:
    return (get_signal(delta[0]), get_signal(delta[1]))


def set_dist_change_mapper(
    delta_0_raw: tuple[float, float], delta_45_raw: tuple[float, float]
) -> None:
    delta_0 = get_signal_delta(delta_0_raw)
    delta_45 = get_signal_delta(delta_45_raw)

    pivot = 0
    reindexed_deltas = []
    for i in range(len(ordered_deltas)):
        if get_signal_delta(ordered_deltas[i]) == delta_0:
            pivot = i
    if (
        get_signal_delta(ordered_deltas[(pivot + 1) % (len(ordered_deltas))])
        == delta_45
    ):
        for j in range(pivot, len(ordered_deltas)):
            reindexed_deltas.append(ordered_deltas[j])
        for j in range(pivot):
            reindexed_deltas.append(ordered_deltas[j])
    else:
        for j in range(pivot, -1, -1):  # j, j-1, j-2...0
            reindexed_deltas.append(ordered_deltas[j])
        for j in range(len(ordered_deltas) - 1, pivot, -1):  # n-1, n-2...pivot+1
            reindexed_deltas.append(ordered_deltas[j])
    for ang in range(0, 360, 45):
        DIST_CHANGE_MAPPER[ang] = reindexed_deltas[int(ang / 45)]
    DIST_CHANGE_MAPPER[360] = DIST_CHANGE_MAPPER[0]


def round_angle(angle: float) -> int:
    angle_degree = angle / DEGREE_IN_RAD
    most_similar = (0, angle_degree)
    for test_angle in POSSIBLE_ANGLES:
        if most_similar[1] > abs(angle_degree - test_angle):
            most_similar = (test_angle, abs(angle_degree - test_angle))
    return most_similar[0]


def expected_gps_after_move(
    initial_position: Coordinate, angle: float, dist: float
) -> Coordinate:
    rounded_angle = round_angle(angle)
    delta = DIST_CHANGE_MAPPER[rounded_angle]
    return Coordinate(
        initial_position.x + dist * delta[0], initial_position.y + dist * delta[1]
    )


class Move(RobotCommand[MovementResult]):
    def __init__(
        self,
        direction: Literal["forward", "backward"],
        dist: float,
        *,
        maze: Maze,
        speed_controller: MovementVelocityController = create_movement_velocity_controller(),
        correction_move: bool = False,
    ):
        """
        Move the robot by certain distance in meters in some direction, using
        the motors.

        :param direction: Direction that robot should move.
        :param dist: Distance that it should try to move.
        :return: The `execute` method returns what was the result of moving.
        - moved
        - hole (in front, left, or right)
        - unavoidable obstacle: If the robot find an obstacle in the path which cannot be avoided.
        For example, because the size between it and a wall is too short.

        OBS:
        - Backward direction is intended only for moves in paths that are guaranteed to not have
        hole, which are not detected in this situation.
        """
        self.direction = direction
        self.dist = dist
        self.maze = maze
        self.speed_controller = speed_controller
        self.correction_move = correction_move

    @log_process(
        [
            "direction",
            "dist",
            "correction_move",
        ],
        System.motor_movement,
        from_self=True,
    )
    def execute(
        self,
        robot: Robot,
    ) -> MovementResult:
        from .recognize_wall_token import RecognizeWallToken
        from .rotate import Rotate

        initial_position = robot.gps.get_position()

        if not self.correction_move:
            imu_expected_angle = cyclic_angle(
                robot.expected_angle
                + (180 * DEGREE_IN_RAD if self.direction == "backward" else 0)
            )
            rounded_angle = round_angle(imu_expected_angle)
            delta = DIST_CHANGE_MAPPER[rounded_angle]
            new_expected_position = expected_gps_after_move(
                robot.expected_position,
                imu_expected_angle,
                self.dist,
            )

        logger.info(
            f"Target: {new_expected_position if not self.correction_move else f'move {self.dist}'} "
            f"from {initial_position=}",
            System.motor_movement,
        )

        found_obstacle = False
        found_wall_token = False

        while robot.step() != -1:
            current_position = robot.gps.get_position()

            is_x_traversed, is_y_traversed = False, False
            if not self.correction_move:
                is_x_traversed = (
                    new_expected_position.x < current_position.x
                    if new_expected_position.x > initial_position.x
                    else new_expected_position.x > current_position.x
                ) or delta[0] == 0
                is_y_traversed = (
                    new_expected_position.y < current_position.y
                    if new_expected_position.y > initial_position.y
                    else new_expected_position.y > current_position.y
                ) or delta[1] == 0

            x_delta = round_if_almost_0(abs(current_position.x - initial_position.x))
            y_delta = round_if_almost_0(abs(current_position.y - initial_position.y))
            traversed_dist = x_delta + y_delta

            logger.debug(
                "Current move status: "
                f"{traversed_dist=}. ({is_x_traversed}; {is_y_traversed}). {current_position=}.",
                System.movement_step_by_step,
            )

            left_velocity, right_velocity = self.speed_controller(
                (
                    max(
                        abs(new_expected_position.x - current_position.x),
                        abs(new_expected_position.y - current_position.y),
                    )
                    if not self.correction_move
                    else self.dist - traversed_dist
                ),
                robot.lidar.get_side_distance(
                    "front" if self.direction == "forward" else "back",
                    field_of_view=20 * DEGREE_IN_RAD,
                    use_min=True,
                ),
                self.direction,
            )
            robot.motor.set_velocity(left_velocity, right_velocity)

            if (
                (is_x_traversed and is_y_traversed)
                if not self.correction_move and not found_obstacle
                else traversed_dist >= self.dist
            ):
                robot.motor.stop()

                if not self.correction_move:
                    robot.expected_position = new_expected_position

                logger.info(
                    f"New position: {current_position}",
                    System.motor_movement,
                )

                break

            left_diagonal_distance = robot.lidar.get_side_distance(
                cyclic_angle(
                    315 * DEGREE_IN_RAD
                    + (180 * DEGREE_IN_RAD if self.direction == "backward" else 0)
                ),
                field_of_view=30 * DEGREE_IN_RAD,
                use_min=True,
            )
            right_diagonal_distance = robot.lidar.get_side_distance(
                cyclic_angle(
                    45 * DEGREE_IN_RAD
                    + (180 * DEGREE_IN_RAD if self.direction == "backward" else 0)
                ),
                field_of_view=30 * DEGREE_IN_RAD,
                use_min=True,
            )
            logger.info(
                f"Diagonal distances: ({left_diagonal_distance}; {right_diagonal_distance})",
                System.obstacle_detection,
            )
            left_diagonal = left_diagonal_distance <= 0.007
            right_diagonal = right_diagonal_distance <= 0.007

            left_side_distance = robot.lidar.get_side_distance(
                cyclic_angle(
                    285 * DEGREE_IN_RAD
                    + (180 * DEGREE_IN_RAD if self.direction == "backward" else 0)
                ),
                field_of_view=30 * DEGREE_IN_RAD,
                use_min=True,
            )
            right_side_distance = robot.lidar.get_side_distance(
                cyclic_angle(
                    75 * DEGREE_IN_RAD
                    + (180 * DEGREE_IN_RAD if self.direction == "backward" else 0)
                ),
                field_of_view=30 * DEGREE_IN_RAD,
                use_min=True,
            )
            logger.info(
                f"Side distances: ({left_diagonal_distance}; {right_side_distance})",
                System.obstacle_detection,
            )
            left_side = left_side_distance <= 0.006
            right_side = right_side_distance <= 0.006

            if (left_diagonal or left_side) and (right_side or right_diagonal):
                logger.info(
                    "Both sides/diagonals with obstacles", System.obstacle_detection
                )

            if (
                (left_diagonal or left_side) and (right_side or right_diagonal)
            ) or robot.lidar.wall_collision(
                "front" if self.direction == "forward" else "back"
            ):
                # TODO-: desfazer movimento obstáculo ou GPS arruma depois já?
                found_obstacle = True
                robot.motor.stop()
                return MovementResult.unavoidable_obstacle
            elif left_diagonal or left_side:
                logger.info("Left obstacle: correction", System.obstacle_avoidance)
                robot.motor.stop()
                robot.run(Rotate(direction="right", angle=PI / 2))
                robot.run(
                    Move(self.direction, 0.001, maze=self.maze, correction_move=True),
                )
                robot.run(Rotate(direction="left", angle=PI / 2))
                found_obstacle = True
            # TODO: tirar obstáculo etc pra área 4
            elif right_diagonal or right_side:
                logger.info("Right obstacle: correction", System.obstacle_avoidance)
                robot.motor.stop()
                robot.run(Rotate(direction="left", angle=PI / 2))
                robot.run(
                    Move(
                        self.direction,
                        0.001,  # TODO: proporcional a quao perto está
                        maze=self.maze,
                        correction_move=True,
                    )
                )
                robot.run(Rotate(direction="right", angle=PI / 2))
                found_obstacle = True

            hole = robot.distance_sensor.detect_hole()
            if (
                hole
                and self.direction == "forward"
                and (
                    self.dist - traversed_dist > DIST_BEFORE_HOLE
                    or self.dist <= DIST_BEFORE_HOLE
                )
                and not self.correction_move
            ):
                robot.motor.stop()
                if hole == "central":
                    return MovementResult.central_hole
                elif hole == "left":
                    return MovementResult.left_hole
                else:
                    return MovementResult.right_hole

            if not found_wall_token and robot.run(RecognizeWallToken()):
                found_wall_token = True

        return MovementResult.moved
