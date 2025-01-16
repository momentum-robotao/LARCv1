import os
import time
from typing import Any, Literal, Protocol

from controller import Robot as WebotsRobot  # type: ignore

from debugging import System, log_process, logger
from devices import (
    GPS,
    IMU,
    Camera,
    ColorSensor,
    Communicator,
    DistanceSensor,
    Lidar,
    Motor,
)
from maze import Maze
from recognize_wall_token import reconhece_lado
from types_and_constants import (
    DEBUG,
    DEGREE_IN_RAD,
    DIST_BEFORE_HOLE,
    EXPECTED_WALL_DISTANCE,
    KP,
    MAX_SPEED,
    PI,
    SLOW_DOWN_ANGLE,
    SLOW_DOWN_DIST,
    SLOW_DOWN_MOVE_SPEED,
    SLOW_DOWN_ROTATE_SPEED,
    TILE_SIZE,
    Coordinate,
    EndOfTimeError,
    LackOfProgressError,
    MovementResult,
    WallColisionError,
)
from utils import cyclic_angle, delay, round_if_almost_0

POSSIBLE_ANGLES = [0, 45, 90, 135, 180, 225, 270, 315, 360]
SQRT_2 = 1.414213562373

DIST_CHANGE_MAPPER: dict[int, tuple[float, float]] = {}

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
        rotation_angle_error: float,
    ) -> tuple[float, float]:
        """
        Speed is `high_speed` if the robot has to move a lot yet
        (more than `slow_down_dist`), else it is `slow_down_speed`.

        :param remaining_distance: The distance that the robot hasn't moved yet.
        :param direction: In which direction the robot is moving.

        :return: A tuple `(left velocity, right velocity)`.
        The signal of those values is defined by `direction`.
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
        rotation_angle_error: float,
    ) -> tuple[float, float]:
        if 2 * abs(rotation_angle_error) > MAX_SPEED:
            logger.warning(
                "No PID, o erro para ser corrigido da rotação do "
                f"robô é: {rotation_angle_error}, maior do que a "
                f"maior velocidade do robô: {MAX_SPEED}",
                System.motor_velocity,
            )

        speed = high_speed
        if remaining_distance <= slow_down_dist:
            speed = slow_down_speed
        elif (
            wall_distance <= wall_distance_if_wall
            and remaining_distance <= slow_down_dist_if_wall
        ):
            speed = slow_down_speed_if_wall

        speed_limit = MAX_SPEED - abs(rotation_angle_error)
        speed = min(speed_limit, speed)

        left_velocity = speed - rotation_angle_error
        right_velocity = speed + rotation_angle_error

        if direction == "backward":
            left_velocity *= -1
            right_velocity *= -1

        return left_velocity, right_velocity

    return movement_velocity_controller


class Robot:
    def __init__(
        self,
        webots_robot: WebotsRobot,
        motor: Motor,
        lidar: Lidar,
        gps: GPS,
        imu: IMU,
        color_sensor: ColorSensor,
        communicator: Communicator,
        camera: Camera,
        distance_sensor: DistanceSensor,
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ):
        self.last_check_time_ms = round(time.time() * 1000)
        self.webots_robot = webots_robot

        self.motor = motor
        self.lidar = lidar
        self.gps = gps
        self.imu = imu
        self.color_sensor = color_sensor
        self.communicator = communicator
        self.camera = camera
        self.distance_sensor = distance_sensor

        self.time_step = time_step

        self.expected_angle = 0.0
        self.step()
        self.expected_position = gps.get_position()
        self.rotating = 0
        # ? 5 primeiros minutos não checa tempo. Depois a cada 2 segs
        self.last_check_time_ms = round(time.time() * 1000) + 5 * 60 * 1000

    def check_time(
        self, time_tolerance: int = int(os.getenv("TIME_TOLERANCE", 3))
    ) -> None:
        game_information = self.communicator.get_game_information()
        if (
            game_information.remaining_real_world_time < time_tolerance
            or game_information.remaining_simulation_time < time_tolerance
        ):
            raise EndOfTimeError()
        self.last_check_time_ms = round(time.time() * 1000)

    def show_initialization_information(self) -> None:
        logger.info(
            "========= Informações do robô inicializado ==========",
            System.initialization,
        )
        if DEBUG:
            logger.info("Modo do robô: teste", System.initialization)

            self.lidar.show_initialization_information()
        else:
            print("Modo do robô: competição")
        logger.info(
            "===================================================\n",
            System.initialization,
        )

    def step(self) -> Any:
        if self.communicator.occured_lack_of_progress():
            raise LackOfProgressError()
        actual_time_ms = round(time.time() * 1000)
        if actual_time_ms - self.last_check_time_ms >= 1000:
            self.check_time()
        return self.webots_robot.step(self.time_step)

    def recognize_wall_token(
        self,
        rotating: bool = False,
        wall_token_approximation: bool = True,
        move_after_found_wall_token: bool = False,
    ) -> bool:
        found = False
        for side, wall_token in [
            (
                "left",
                reconhece_lado(
                    self.camera._left_camera,
                    "left",
                    self.lidar,
                    rotating=rotating,
                ),
            ),
            (
                "right",
                reconhece_lado(
                    self.camera._right_camera,
                    "right",
                    self.lidar,
                    rotating=rotating,
                ),
            ),
        ]:
            if wall_token:
                self.motor.stop()
                if wall_token_approximation:
                    to_move = 0.0
                    if side == "left":
                        to_move = self.lidar.get_side_distance(
                            "left", use_min=True, field_of_view=40 * DEGREE_IN_RAD
                        )
                        self.rotate_90_left(just_rotate=True)
                    if side == "right":
                        to_move = self.lidar.get_side_distance(
                            "right", use_min=True, field_of_view=40 * DEGREE_IN_RAD
                        )
                        self.rotate_90_right(just_rotate=True)
                    logger.info("go to wall token", System.movement_reason)
                    self.move(
                        "forward",
                        to_move / 2,
                        maze=Maze(),
                        just_move=True,
                    )  # TODO-: test Maze
                delay(self.webots_robot, 1300)
                self.communicator.send_wall_token_information(
                    self.gps.get_position(), wall_token
                )
                if wall_token_approximation:
                    logger.info("return from wall token", System.movement_reason)
                    self.move(
                        "backward",
                        to_move / 2,
                        maze=Maze(),
                        just_move=True,
                    )  # TODO-: test Maze
                    if side == "left":
                        self.rotate_90_right(just_rotate=True)
                    if side == "right":
                        self.rotate_90_left(just_rotate=True)
                if move_after_found_wall_token:
                    self.motor.set_velocity(6.24 - 3.5, 6.24 - 3.5)
                    delay(self.webots_robot, 100)
                    self.motor.stop()
                found = True
        return found

    def rotate(
        self,
        direction: Literal["left", "right"],
        turn_angle: float,
        *,
        correction_rotation: bool = False,
        speed_controller: RotationVelocityController = create_rotation_velocity_controller(),
        just_rotate: bool = False,
        dfs_rotation: bool = True,
    ) -> None:
        """
        Rotate the robot in a direction by an angle, using the motors. Uses
        `imu` to check the robot angle to rotate correctly.
        """
        was_rotating = self.rotating > 0
        self.rotating += 1
        rotation_angle = self.imu.get_rotation_angle()

        logger.info(
            f"     rotacionando {turn_angle / DEGREE_IN_RAD} para {direction}. "
            "Era {self.expected_angle / DEGREE_IN_RAD}",
            System.rotation,
        )
        recognized_wall_token = False
        if not correction_rotation and not was_rotating and dfs_rotation:
            self.expected_angle = cyclic_angle(
                self.expected_angle + (-1 if direction == "left" else 1) * turn_angle
            )
            for test_angle_degree in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
                test_angle = test_angle_degree * DEGREE_IN_RAD
                if abs(test_angle - self.expected_angle) <= 0.05:
                    self.expected_angle = test_angle
            if direction == "left":  # changed
                if self.expected_angle > rotation_angle:
                    turn_angle = 2 * PI - (self.expected_angle - rotation_angle)
                else:
                    turn_angle = rotation_angle - self.expected_angle
            else:
                if self.expected_angle > rotation_angle:
                    turn_angle = self.expected_angle - rotation_angle
                else:
                    turn_angle = 2 * PI - (rotation_angle - self.expected_angle)

        self.motor.stop()

        angle_accumulated_delta = 0

        while self.step() != -1:
            if (
                not just_rotate
                and not recognized_wall_token
                and self.recognize_wall_token(rotating=True)
            ):
                recognized_wall_token = True

            new_robot_angle = self.imu.get_rotation_angle()
            angle_accumulated_delta += IMU.get_delta_rotation(
                rotation_angle, new_robot_angle
            )
            angle_to_rotate = turn_angle - angle_accumulated_delta

            logger.info(
                f"- Já girou {angle_accumulated_delta} no total, falta "
                f"{angle_to_rotate}",
                System.rotation,
            )

            rotation_angle = new_robot_angle

            left_velocity, right_velocity = speed_controller(angle_to_rotate, direction)
            self.motor.set_velocity(left_velocity, right_velocity)

            if angle_accumulated_delta >= turn_angle:
                self.motor.stop()

                logger.info(
                    f"=== Terminou de girar {angle_accumulated_delta} "
                    f"para {direction}. ",
                    System.rotation,
                )

                logger.info(
                    f"Girou a mais {(angle_accumulated_delta - turn_angle)/DEGREE_IN_RAD}",
                    System.rotation_angle_correction,
                )
                if angle_accumulated_delta - turn_angle >= 0.1:  # changed
                    self.rotate(
                        "left" if direction == "right" else "right",
                        angle_accumulated_delta - turn_angle,
                        correction_rotation=True,
                        speed_controller=create_rotation_velocity_controller(
                            slow_down_angle=angle_accumulated_delta - turn_angle + 1
                        ),
                    )
                    logger.info(
                        f"Girou demais, voltou {self.expected_angle=}. "
                        f"Parando em: {self.imu.get_rotation_angle()}",
                        System.rotation_angle_correction,
                    )

                break
        self.rotating -= 1

    def rotate_180(self) -> None:
        """Rotate 180 degrees."""
        self.rotate("right", PI)

    def rotate_90_left(self, just_rotate=False) -> None:
        """Rotate 90 degrees to left."""
        self.rotate("left", PI / 2, just_rotate=just_rotate)

    def rotate_90_right(self, just_rotate=False) -> None:
        """Rotate 90 degrees to right."""
        self.rotate("right", PI / 2, just_rotate=just_rotate)

    def rotate_to_angle(
        self,
        angle: float,
        speed_controller: RotationVelocityController = create_rotation_velocity_controller(),
    ) -> None:
        """
        Rotate the robot to `angle`. It rotates to the direction that
        makes this rotation faster.
        """
        robot_rotation_angle = self.imu.get_rotation_angle()
        angle_rotating_right = cyclic_angle(2 * PI + angle - robot_rotation_angle)
        angle_rotating_left = 2 * PI - angle_rotating_right  # complementar angles
        if angle_rotating_right <= PI:
            self.rotate(
                "right",
                angle_rotating_right,
                speed_controller=speed_controller,
            )
        else:
            self.rotate(
                "left",
                angle_rotating_left,
                speed_controller=speed_controller,
            )

    @log_process(
        [
            "direction",
            "dist",
            "returning_to_safe_position",
            "correction_move",
            "just_move",
            "dfs_move",
        ],
        System.motor_movement,
    )
    def move(
        self,
        direction: Literal["forward", "backward"],
        dist: float,
        *,
        maze: Maze,
        speed_controller: MovementVelocityController = create_movement_velocity_controller(),
        kp: float = KP,
        expected_wall_distance: float = EXPECTED_WALL_DISTANCE,
        returning_to_safe_position: bool = False,
        correction_move: bool = False,
        just_move: bool = False,  # TODO: refactor all these cases
        dfs_move: bool = True,
    ) -> MovementResult:
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
        - Holes are not detected when moving backward.
        """
        found_wall_token = False
        initial_position = self.gps.get_position()

        if not correction_move and dfs_move and not just_move:
            initial_expected_position = self.expected_position
            imu_expected_angle = cyclic_angle(
                self.expected_angle
                + (180 * DEGREE_IN_RAD if direction == "backward" else 0)
            )

            rounded_angle = round_angle(imu_expected_angle)
            delta = DIST_CHANGE_MAPPER[rounded_angle]
            if not returning_to_safe_position:
                self.expected_position = expected_gps_after_move(
                    self.expected_position,
                    imu_expected_angle,
                    dist,
                )

        logger.info(
            f"Target: {self.expected_position} from {initial_position=}",
            System.motor_movement,
        )

        found_obstacle = False
        found_hole_type = None
        blocking = False

        while self.step() != -1:
            current_position = self.gps.get_position()

            rotation_angle_error = self.lidar.get_rotation_angle_error(
                expected_wall_distance, kp
            )

            left_velocity, right_velocity = speed_controller(
                max(
                    abs(self.expected_position.x - current_position.x),
                    abs(self.expected_position.y - current_position.y),
                ),
                self.lidar.get_side_distance(
                    "front" if direction == "forward" else "back",
                    field_of_view=20 * DEGREE_IN_RAD,
                    use_min=True,
                ),
                direction,
                rotation_angle_error,
            )
            self.motor.set_velocity(left_velocity, right_velocity)

            is_x_traversed, is_y_traversed = False, False
            if not correction_move and dfs_move and not just_move:
                is_x_traversed = (
                    self.expected_position.x < current_position.x
                    if self.expected_position.x > initial_position.x
                    else self.expected_position.x > current_position.x
                ) or delta[0] == 0
                is_y_traversed = (
                    self.expected_position.y < current_position.y
                    if self.expected_position.y > initial_position.y
                    else self.expected_position.y > current_position.y
                ) or delta[1] == 0

            if dfs_move and not just_move:
                left_diagonal_distance = self.lidar.get_side_distance(
                    cyclic_angle(
                        315 * DEGREE_IN_RAD
                        + (180 * DEGREE_IN_RAD if direction == "backward" else 0)
                    ),
                    field_of_view=30 * DEGREE_IN_RAD,
                    use_min=True,
                )
                right_diagonal_distance = self.lidar.get_side_distance(
                    cyclic_angle(
                        45 * DEGREE_IN_RAD
                        + (180 * DEGREE_IN_RAD if direction == "backward" else 0)
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

                left_side_distance = self.lidar.get_side_distance(
                    cyclic_angle(
                        285 * DEGREE_IN_RAD
                        + (180 * DEGREE_IN_RAD if direction == "backward" else 0)
                    ),
                    field_of_view=30 * DEGREE_IN_RAD,
                    use_min=True,
                )
                right_side_distance = self.lidar.get_side_distance(
                    cyclic_angle(
                        75 * DEGREE_IN_RAD
                        + (180 * DEGREE_IN_RAD if direction == "backward" else 0)
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
                    (left_diagonal or left_side)
                    and (right_side or right_diagonal)
                    and not just_move
                ):
                    # TODO-: desfazer movimento obstáculo ou GPS arruma depois já?
                    found_obstacle = True
                    blocking = True
                    break

                if left_diagonal or left_side:
                    logger.info("Left obstacle: correction", System.obstacle_avoidance)
                    self.motor.stop()
                    self.rotate_90_right()
                    self.move(
                        direction,
                        0.001,
                        maze=maze,
                        correction_move=True,
                    )
                    self.rotate_90_left()
                    found_obstacle = True
                # TODO: tirar obstáculo etc pra área 4
                if right_diagonal or right_side:
                    logger.info("Right obstacle: correction", System.obstacle_avoidance)
                    self.motor.stop()
                    self.rotate_90_left()
                    self.move(
                        direction,
                        0.001,  # TODO: proporcional a quao perto está
                        maze=maze,
                        correction_move=True,
                    )
                    self.rotate_90_right()
                    found_obstacle = True

            x_delta = round_if_almost_0(abs(current_position.x - initial_position.x))
            y_delta = round_if_almost_0(abs(current_position.y - initial_position.y))
            traversed_dist = x_delta + y_delta

            logger.debug(
                "Current move status: "
                f"{traversed_dist=}. ({is_x_traversed}; {is_y_traversed}). {current_position=}.",
                System.movement_step_by_step,
            )

            if (
                (is_x_traversed and is_y_traversed)
                if not correction_move
                and not found_obstacle
                and dfs_move
                and not just_move
                else traversed_dist >= dist
            ):
                self.motor.stop()

                logger.info(
                    f"New position: {current_position}",
                    System.motor_movement,
                )

                break

            if self.lidar.wall_collision("front" if direction == "forward" else "back"):
                if not returning_to_safe_position and dfs_move and not just_move:
                    blocking = True

                    logger.info(
                        "Detected collision: will return to last position",
                        System.lidar_wall_detection,
                    )

                    break
                else:
                    logger.warning("would be a collision", System.lidar_wall_detection)

            hole = self.distance_sensor.detect_hole()
            if (
                hole
                and not returning_to_safe_position
                and (
                    dist - traversed_dist > DIST_BEFORE_HOLE or dist <= DIST_BEFORE_HOLE
                )
                and dfs_move
                and not correction_move
                and not just_move
            ):
                logger.info(
                    "Retornará à posição antiga após achar buraco.",
                    System.hole_detection,
                )

                blocking = True
                if hole == "central":
                    found_hole_type = MovementResult.central_hole
                elif hole == "left":
                    found_hole_type = MovementResult.left_hole
                else:
                    found_hole_type = MovementResult.right_hole
                break

            if not just_move and not found_wall_token and self.recognize_wall_token():
                found_wall_token = True

        if blocking:  # ? hole, obstacle or unexpected wall collision
            self.motor.stop()
            self.expected_position = initial_expected_position
            logger.info("Blocked: return to initial position", System.movement_reason)
            self.move(
                "backward" if direction == "forward" else "forward",
                traversed_dist,
                maze=maze,
                speed_controller=speed_controller,
                kp=kp,
                expected_wall_distance=expected_wall_distance,
                returning_to_safe_position=True,
            )
            if found_hole_type:
                return found_hole_type
            if DEBUG and found_obstacle:
                # TODO: deixar 'branco' no mapa
                pass
            raise WallColisionError()

        return MovementResult.moved
