import os
from typing import Any, Literal

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
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
from helpers import cyclic_angle, delay, round_if_almost_0
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
    SLOW_DOWN_DIST,
    SLOW_DOWN_SPEED,
    TILE_SIZE,
    Coordinate,
    EndOfTimeError,
    MovementResult,
    WallColisionError,
)

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
    dist_to_move: float,
    direction: Literal["forward", "backward"],
    rotation_angle_error: float,
    debug_info: DebugInfo,
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
        debug_info.send(
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
        debug_info: DebugInfo,
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ):
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
        self.debug_info = debug_info

        self.expected_angle = 0.0
        self.step()
        self.expected_position = gps.get_position()

    def show_initialization_information(self) -> None:
        self.debug_info.send(
            "========= Informações do robô inicializado ==========",
            System.initialization,
        )
        if DEBUG:
            self.debug_info.send("Modo do robô: teste", System.initialization)

            self.lidar.show_initialization_information()
        else:
            self.debug_info.send("Modo do robô: competição", System.initialization)
        self.debug_info.send(
            "===================================================\n",
            System.initialization,
        )

    def step(self) -> Any:
        return self.webots_robot.step(self.time_step)

    def recognize_wall_token(self) -> None:
        for wall_token in [
            reconhece_lado(
                self.camera._left_camera, self.debug_info, "left", self.lidar
            ),
            reconhece_lado(
                self.camera._right_camera, self.debug_info, "right", self.lidar
            ),
        ]:
            if wall_token:
                self.motor.stop()
                delay(self.webots_robot, self.debug_info, 1300)
                self.communicator.send_wall_token_information(
                    self.gps.get_position(), wall_token
                )

    def rotate(
        self,
        direction: Literal["left", "right"],
        turn_angle: float,
        correction_rotation: bool = False,
        slow_down_angle: float = 0.1,
        high_speed: float = MAX_SPEED,
        low_speed: float = MAX_SPEED / 100,
    ) -> None:
        """
        Rotate the robot in a direction by an angle, using the motors. Uses
        `imu` to check the robot angle to rotate correctly.
        """

        if not correction_rotation:
            self.expected_angle = cyclic_angle(
                self.expected_angle + (-1 if direction == "left" else 1) * turn_angle
            )
        self.motor.stop()

        angle_accumulated_delta = 0
        rotation_angle = self.imu.get_rotation_angle()

        while self.step() != -1:
            new_robot_angle = self.imu.get_rotation_angle()
            angle_accumulated_delta += IMU.get_delta_rotation(
                rotation_angle, new_robot_angle
            )
            angle_to_rotate = turn_angle - angle_accumulated_delta
            print("Já girou: ", angle_accumulated_delta)

            if DEBUG:
                self.debug_info.send(
                    f"- Já girou {angle_accumulated_delta} no total, falta "
                    f"{angle_to_rotate}",
                    System.motor_rotation,
                )

            rotation_angle = new_robot_angle

            left_velocity, right_velocity = rotation_velocity_controller(
                angle_to_rotate, direction, slow_down_angle, high_speed, low_speed
            )
            self.motor.set_velocity(left_velocity, right_velocity)

            if angle_accumulated_delta >= turn_angle:
                self.motor.stop()

                if DEBUG:
                    self.debug_info.send(
                        f"=== Terminou de girar {angle_accumulated_delta} "
                        f"para {direction}. ",
                        System.motor_rotation,
                    )

                break

    def rotate_180(self) -> None:
        """Rotate 180 degrees."""
        self.rotate("right", PI)

    def rotate_90_left(self) -> None:
        """Rotate 90 degrees to left."""
        self.rotate("left", PI / 2)

    def rotate_90_right(self) -> None:
        """Rotate 90 degrees to right."""
        self.rotate("right", PI / 2)

    def rotate_to_angle(
        self,
        angle: float,
        slow_down_angle: float = 0.1,
        high_speed: float = MAX_SPEED,
        low_speed: float = MAX_SPEED / 100,
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
                slow_down_angle=slow_down_angle,
                high_speed=high_speed,
                low_speed=low_speed,
            )
        else:
            self.rotate(
                "left",
                angle_rotating_left,
                slow_down_angle=slow_down_angle,
                high_speed=high_speed,
                low_speed=low_speed,
            )

    def move(
        self,
        direction: Literal["forward", "backward"],
        maze: Maze,
        *,
        dist: float = TILE_SIZE,
        slow_down_dist: float = SLOW_DOWN_DIST,
        high_speed: float = MAX_SPEED,
        slow_down_speed: float = SLOW_DOWN_SPEED,
        kp: float = KP,
        expected_wall_distance: float = EXPECTED_WALL_DISTANCE,
        returning_to_safe_position: bool = False,
        correction_move: bool = False,
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
        initial_position = self.gps.get_position()
        # if self.expected_raw_angle is None:
        #     self.expected_raw_angle = imu.get_rotation_angle(raw=True)
        if not correction_move:
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

        if DEBUG:
            self.debug_info.send(
                f"Começando a mover {dist} para {direction}. Chegará: {self.expected_position}",
                System.motor_movement,
            )

        found_obstacle = False

        while self.step() != -1:
            actual_position = self.gps.get_position()

            rotation_angle_error = self.lidar.get_rotation_angle_error(
                expected_wall_distance, kp
            )

            if DEBUG:
                self.debug_info.send(
                    f"Já moveu até {actual_position}.",
                    System.motor_movement,
                )

            left_velocity, right_velocity = movement_velocity_controller(
                max(
                    abs(self.expected_position.x - actual_position.x),
                    abs(self.expected_position.y - actual_position.y),
                ),
                direction,
                rotation_angle_error,
                self.debug_info,
                slow_down_dist,
                high_speed,
                slow_down_speed,
            )
            self.motor.set_velocity(left_velocity, right_velocity)

            x_traversed, y_traversed = False, False
            if not correction_move:
                x_traversed = (
                    self.expected_position.x < actual_position.x
                    if self.expected_position.x > initial_position.x
                    else self.expected_position.x > actual_position.x
                ) or delta[0] == 0
                y_traversed = (
                    self.expected_position.y < actual_position.y
                    if self.expected_position.y > initial_position.y
                    else self.expected_position.y > actual_position.y
                ) or delta[1] == 0

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
            print("diagonais", left_diagonal_distance, right_diagonal_distance)
            left_diagonal = left_diagonal_distance <= 0.004
            right_diagonal = right_diagonal_distance <= 0.004

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
            print("laterais", left_side_distance, right_side_distance)
            left_side = left_side_distance <= 0.008
            right_side = right_side_distance <= 0.008

            if (left_diagonal or left_side) and (right_side or right_diagonal):
                # TODO: retornar para posição livre e desfazer movimento obstáculo
                raise WallColisionError()

            if left_diagonal or left_side:
                self.motor.stop()
                self.rotate_90_right()
                self.move(
                    direction,
                    maze,
                    dist=0.001,
                    correction_move=True,
                )
                self.rotate_90_left()
                found_obstacle = True

            if right_diagonal or right_side:
                self.motor.stop()
                self.rotate_90_left()
                self.move(
                    direction,
                    maze,
                    dist=0.001,  # TODO: proporcional a quao perto está
                    correction_move=True,
                )
                self.rotate_90_right()
                found_obstacle = True

            front_distance = self.lidar.get_side_distance(
                180 * DEGREE_IN_RAD if direction == "backward" else 0,
                field_of_view=30 * DEGREE_IN_RAD,
                use_min=True,
            )
            if front_distance < 0.01:
                self.motor.stop()
                found_obstacle = True
                return MovementResult.moved

            if found_obstacle:
                print("TODO: deixar 'branco' no mapa")

            x_delta = round_if_almost_0(abs(actual_position.x - initial_position.x))
            y_delta = round_if_almost_0(abs(actual_position.y - initial_position.y))
            traversed_dist = x_delta + y_delta

            if (
                (x_traversed and y_traversed)
                if not correction_move and not found_obstacle
                else traversed_dist >= dist
            ):
                self.motor.stop()

                if DEBUG:
                    self.debug_info.send(
                        f"Fim do movimento, andou para {actual_position}",
                        System.motor_movement,
                    )

                break

            if (
                self.lidar.wall_collision("front" if direction == "forward" else "back")
                and not returning_to_safe_position
            ):
                self.motor.stop()
                self.expected_position = initial_expected_position
                self.move(
                    "backward" if direction == "forward" else "forward",
                    maze,
                    dist=traversed_dist,
                    slow_down_dist=slow_down_dist,
                    high_speed=high_speed,
                    slow_down_speed=slow_down_speed,
                    kp=kp,
                    expected_wall_distance=expected_wall_distance,
                    returning_to_safe_position=True,  # TODO: lidar com esse caso na dfs como parede e returning_to_safe_position desfazer movimento que é feito para obstáculo
                )

                if DEBUG:
                    self.debug_info.send(
                        "Retornou à posição antiga após colidir com parede.",
                        System.lidar_wall_detection,
                    )

                raise WallColisionError()

            hole = self.distance_sensor.detect_hole()
            if (
                hole
                and not returning_to_safe_position
                and (
                    dist - traversed_dist > DIST_BEFORE_HOLE or dist <= DIST_BEFORE_HOLE
                )
            ):
                self.motor.stop()
                self.expected_position = initial_expected_position

                self.move(
                    "backward" if direction == "forward" else "forward",
                    maze,
                    dist=traversed_dist,
                    slow_down_dist=slow_down_dist,
                    high_speed=high_speed,
                    slow_down_speed=slow_down_speed,
                    kp=kp,
                    expected_wall_distance=expected_wall_distance,
                    returning_to_safe_position=True,
                )

                if DEBUG:
                    self.debug_info.send(
                        "Retornou à posição antiga após achar buraco.",
                        System.hole_detection,
                    )

                if hole == "central":
                    return MovementResult.central_hole
                elif hole == "left":
                    return MovementResult.left_hole
                else:
                    return MovementResult.right_hole

            self.recognize_wall_token()
        return MovementResult.moved


def check_time(
    robot: Robot, time_tolerance: int = int(os.getenv("TIME_TOLERANCE", 3))
) -> None:
    game_information = robot.communicator.get_game_information()
    if (
        game_information.remaining_real_world_time < time_tolerance
        or game_information.remaining_simulation_time < time_tolerance
    ):
        raise EndOfTimeError()
