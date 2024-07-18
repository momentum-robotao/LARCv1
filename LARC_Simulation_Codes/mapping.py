import logging
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Literal, Union

from controller import Robot

PI = 3.14159265359
DEGREE_IN_RAD = 0.0174533

SLOW_DOWN_SPEED = 0.1
SLOW_DOWN_DIST = 0.001
MAX_SPEED = 6.28
KP = 50.0
TILE_SIZE = 0.12
WALL_TICKNESS = 0.01
ROBOT_RADIUS = 0.0355
MAX_WALL_DISTANCE = 0.030
EXPECTED_WALL_DISTANCE = 0.019
WALL_COLLISION_DISTANCE = 0.002

HOLE_COLOR = b"...\xff"

# TODO: correction deve ser baseada em qual eixo está, o que começou ou perpendicular ao inicial

# Initialize logger
try:
    log_dir = os.path.dirname(os.getenv("LOG_PATH", ""))
    os.makedirs(log_dir, exist_ok=True)
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(levelname)s %(message)s",
        filename=os.getenv("LOG_PATH"),
    )
    logger = logging.getLogger("Robo LARC v1")
except Exception:
    if os.getenv("DEBUG"):
        logging.error("Erro ao inicializar o logger", exc_info=True)
        raise


class WallColisionError(Exception):
    """
    Exception raised when the robot colides with a wall and it
    has already returned to its last position.
    """

    pass


class MovementConsequence(Enum):
    moved = "moved"
    hole = "hole"


class System(Enum):
    delay = "delay"
    lidar_general = "lidar general"
    lidar_measures = "lidar measures"
    lidar_range_measures = "lidar range measures"
    lidar_side_measures = "lidar side measures"
    lidar_wall_detection = "lidar wall detection"
    gps_measures = "gps measures"
    imu_measures = "imu measures"
    motor_rotation = "motor rotation"
    motor_movement = "motor movement"
    motor_velocity = "motor velocity"
    rotation_angle_correction = "rotation angle correction"
    color_sensor_measures = "color sensor measures"
    color_sensor_detections = "color sensor detections"


ALL_SYSTEMS = [system for system in System]


class Victim(Enum):
    HARMED = "H"
    STABLE = "S"
    UNHARMED = "U"


class HazmatSign(Enum):
    FLAMMABLE_GAS = "F"
    POISON = "P"
    CORROSIVE = "C"
    ORGANIC_PEROXIDE = "O"


WallToken = Union[HazmatSign, Victim]


class Wall(Enum):
    NONEXISTENT = 0
    EXISTENT = 1


class TileType(Enum):
    AREA_4 = -1
    EMPTY = 0
    HOLE = 2
    SWAMP = 3
    CHECKPOINT = 4
    STARTING = 5
    CONNECTION_1_TO_2 = 6
    CONNECTION_2_TO_3 = 7
    CONNECTION_3_TO_4 = 8
    CONNECTION_1_TO_4 = 9


@dataclass
class QuarterTile:
    up: Wall
    down: Wall
    left: Wall
    right: Wall


@dataclass
class Tile:
    top_left: QuarterTile
    top_right: QuarterTile
    bottom_left: QuarterTile
    bottom_right: QuarterTile
    type: TileType


@dataclass
class Coordinate:
    x: float
    y: float
    z: float


@dataclass
class RGB:
    red: float
    green: float
    blue: float


class DebugInfo:
    def __init__(self, systems_to_debug: Union[list[System], None] = None) -> None:
        self.systems_to_debug = [] if systems_to_debug is None else systems_to_debug
        if not os.getenv("DEBUG"):
            self.systems_to_debug = []

    def send(self, message: str, system_being_debugged: System) -> None:
        if system_being_debugged not in self.systems_to_debug:
            if os.getenv("DEBUG"):
                logger.debug(f"{system_being_debugged} => {message}")
            return
        logger.info(f"{system_being_debugged} => {message}")
        print(f"{system_being_debugged} => {message}")


def round_if_almost_0(value: float):
    return 0 if round(value, 2) == 0 else value


def delay(
    robot,
    debug_info: DebugInfo,
    time_ms: Union[float, int],
    time_step: int = int(os.getenv("TIME_STEP", 32)),
):
    if os.getenv("DEBUG"):
        debug_info.send(f"Esperando: {time_ms}ms", System.delay)

    init_time = robot.getTime()
    while robot.step(time_step) != -1:
        time_elapsed_ms = (robot.getTime() - init_time) * 1000.0
        if time_elapsed_ms > time_ms:
            break


class Device(ABC):
    """
    Classes responsible for interacting with the robot, getting data from
    their sensors and sending commands to it.
    """

    @abstractmethod
    def __init__(self, robot, debug_info: DebugInfo, *args, **kwargs) -> None:
        pass


class Lidar(Device):
    def __init__(
        self,
        robot,
        debug_info: DebugInfo,
        lidar_name: str = "lidar",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._lidar = robot.getDevice(lidar_name)
        self._lidar.enable(time_step)

        # LIDAR attributes
        self.horizontal_resolution = self._lidar.getHorizontalResolution()
        self.number_of_layers = self._lidar.getNumberOfLayers()
        self.min_range = self._lidar.getMinRange()
        self.max_range = self._lidar.getMaxRange()
        self.field_of_view = self._lidar.getFov()

        self.debug_info = debug_info

    def _get_line_distances(self, line: int) -> list[float]:
        """
        :return: A list with the distances measured by lidar in the
        corresponding line.
        """
        return self._lidar.getLayerRangeImage(line).copy()

    def _get_measure_angle(self, measure_idx: int) -> float:
        angle = measure_idx * (self.field_of_view / self.horizontal_resolution)
        return angle

    def get_distances(self) -> list[float]:
        """
        Gets the distance in each angle of the robot measured in
        meters by lidar and corresponds to the distances from the
        robot sides.

        OBS:
        - Zero angle depends of the value set in the robot file
        (probably, it is the front of the robot)
        - The angle of the measurements increase clockwise.
        """
        # Gets minimum to get the most similar to perpendicular measures
        distances = [float("inf") for _ in range(self.horizontal_resolution)]
        for line in range(self.number_of_layers):
            line_distances = self._get_line_distances(line)
            for col in range(self.horizontal_resolution):
                distances[col] = min(distances[col], line_distances[col] - ROBOT_RADIUS)

        if os.getenv("DEBUG"):
            self.debug_info.send(f"{distances=}", System.lidar_measures)
        return distances

    def get_distances_by_angle(self) -> Dict[float, float]:
        """
        :return: A dict where `dict[angle] = distance measured in this angle`.
        """
        distances = self.get_distances()
        distances_by_angle = dict()
        for measure_idx, dist in enumerate(distances):
            angle = round(self._get_measure_angle(measure_idx), 2)
            distances_by_angle[angle] = dist

        if os.getenv("DEBUG"):
            debug_info.send(
                f"Medições da distância em função do ângulo: {distances_by_angle}",
                System.lidar_measures,
            )

        return distances_by_angle

    def get_distances_of_range(
        self, initial_angle: float, end_angle: float
    ) -> list[float]:
        """
        Gets the distance in each angle of the robot measured in
        meters by lidar and corresponds to the distances from the
        robot sides. The measures corresponds only to the ones of
        the angles of specified range.

        OBS:
        - Zero angle depends of the value set in the robot file
        (probably, it is the front of the robot)
        - If `end_angle < initial_angle`, the range of measures are
        get from angles increasing and coming back to angle 0 and then
        increasing until `initial_angle`.
        Ex: if angles of measures were [1, 2, 3, 4, 5], `initial_angle = 4`
        and `end_angle = 2`, the measures would correspond to these angles:
        [4, 5, 1, 2]
        - The angle of the measurements increase clockwise.
        """
        distances_by_angle = self.get_distances_by_angle()
        distances_of_range = []
        if initial_angle <= end_angle:
            for angle, dist in distances_by_angle.items():
                if initial_angle <= angle and angle <= end_angle:
                    distances_of_range.append(dist)
        else:
            for angle, dist in distances_by_angle.items():
                if initial_angle <= angle:
                    distances_of_range.append(dist)
            for angle, dist in distances_by_angle.items():
                if angle <= end_angle:
                    distances_of_range.append(dist)

        if os.getenv("DEBUG"):
            debug_info.send(
                f"Pegas medições do intervalo cíclico baseado nos ângulos: [{initial_angle};{end_angle}]. "
                f"Medições: {distances_of_range}",
                System.lidar_range_measures,
            )

        return distances_of_range

    def get_side_distance(
        self,
        side: Literal["front", "back", "left", "right"],
        field_of_view: float = 10 * DEGREE_IN_RAD,
    ) -> float:
        """
        :return: The average distance from `field_of_view` centralized in
        a side of the robot.
        """
        distances = []
        if side == "front":
            distances = self.get_distances_of_range(
                360 * DEGREE_IN_RAD - field_of_view / 2, field_of_view / 2
            )
        elif side == "right":
            distances = self.get_distances_of_range(
                90 * DEGREE_IN_RAD - field_of_view / 2,
                90 * DEGREE_IN_RAD + field_of_view / 2,
            )
        elif side == "back":
            distances = self.get_distances_of_range(
                180 * DEGREE_IN_RAD - field_of_view / 2,
                180 * DEGREE_IN_RAD + field_of_view / 2,
            )
        elif side == "left":
            distances = self.get_distances_of_range(
                270 * DEGREE_IN_RAD - field_of_view / 2,
                270 * DEGREE_IN_RAD + field_of_view / 2,
            )

        average_distance = sum(distances) / len(distances)

        if os.getenv("DEBUG"):
            self.debug_info.send(
                f"Medidas correspondentes à {side} "
                f"(com campo de visão de {field_of_view} rad): {distances}; "
                f"Dando média de: {average_distance}m",
                System.lidar_side_measures,
            )

        return average_distance

    def has_wall(
        self,
        side: Literal["front", "back", "left", "right"],
        max_wall_distance: float = MAX_WALL_DISTANCE,
    ) -> bool:
        """
        :return: If there is a wall in the actual tile, assuming that
        the robot is on the center of the tile on the corresponding axis.
        """
        side_distance = self.get_side_distance(side)
        has_wall = side_distance <= max_wall_distance

        if os.getenv("DEBUG"):
            self.debug_info.send(
                f"{'Tem' if has_wall else 'Não tem'} parede em {side}. "
                f"Limite de parede: {max_wall_distance}.",
                System.lidar_wall_detection,
            )

        return has_wall

    def wall_collision(
        self,
        side: Literal["front", "back"],
        wall_collision_dist: float = WALL_COLLISION_DISTANCE,
    ) -> bool:
        wall_dist = self.get_side_distance(side)
        wall_collision = wall_dist <= wall_collision_dist
        if os.getenv("DEBUG"):
            if wall_collision:
                self.debug_info.send(
                    f"Colisão com parede detectada há: {wall_dist}m",
                    System.lidar_wall_detection,
                )
        return wall_collision

    def get_rotation_angle_error(
        self,
        expected_wall_distance: float = EXPECTED_WALL_DISTANCE,
        kp: float = KP,
    ) -> float:
        rotation_angle_error = 0.0
        if lidar.has_wall("left"):
            rotation_angle_error = (
                lidar.get_side_distance("left") - expected_wall_distance
            ) * kp
        if lidar.has_wall("right"):
            rotation_angle_error = (
                (lidar.get_side_distance("right") - expected_wall_distance) * kp * -1
            )

        if os.getenv("DEBUG"):
            self.debug_info.send(
                "Erro do ângulo de rotação do robô para ser corrigido: "
                f"{rotation_angle_error}. Com {kp=}, com distância alvo "
                f"da parede de: {expected_wall_distance}",
                System.rotation_angle_correction,
            )

        return rotation_angle_error


class GPS(Device):
    def __init__(
        self,
        robot,
        debug_info: DebugInfo,
        gps_name: str = "gps",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self._gps = robot.getDevice(gps_name)
        self._gps.enable(time_step)

        self.debug_info = debug_info

    def get_coordinates(self) -> Coordinate:
        positions = self._gps.getValues()
        x, z, y = positions
        coordinate = Coordinate(x, y, z)
        if os.getenv("DEBUG"):
            self.debug_info.send(str(coordinate), System.gps_measures)
        return coordinate


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

    def get_rotation_angle(self) -> float:
        rotation_angle = self._imu.getRollPitchYaw()[2]
        if os.getenv("DEBUG"):
            self.debug_info.send(
                f"Ângulo do robô: {rotation_angle}", System.imu_measures
            )
        return rotation_angle

    @staticmethod
    def get_delta_rotation(ang: float, new_ang: float):
        """
        Get delta of an angle.

        WARNING! The angle must have changed just a little, as this
        assumption is used to calculate the delta of the angle. It
        is recommended to the change corresponds to only a time_step rotation
        """
        if new_ang * ang <= 0:
            if round(ang) == 0:
                return abs(ang) + abs(new_ang)
            else:
                return abs(abs(ang) - PI) + abs(abs(new_ang) - PI)
        return abs(ang - new_ang)


class ColorSensor(Device):
    def __init__(
        self,
        robot,
        debug_info: DebugInfo,
        color_sensor_name: str = "colour_sensor",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ) -> None:
        self.debug_info = debug_info

        self._color_sensor = robot.getDevice(color_sensor_name)
        self._color_sensor.enable(time_step)

    def get_color(self) -> bytes:
        color = self._color_sensor.getImage()
        if os.getenv("DEBUG"):
            self.debug_info.send(
                f"Cor reconhecida: {color}", System.color_sensor_measures
            )
        return color

    def get_RGB_color(self) -> RGB:
        image = self._color_sensor.getImage()

        red = self._color_sensor.imageGetRed(image, 1, 0, 0)
        green = self._color_sensor.imageGetGreen(image, 1, 0, 0)
        blue = self._color_sensor.imageGetBlue(image, 1, 0, 0)
        color = RGB(red, green, blue)
        if os.getenv("DEBUG"):
            self.debug_info.send(
                f"Cor RGB reconhecida: {color}", System.color_sensor_measures
            )
        return color

    def has_hole(self, hole_color: bytes = HOLE_COLOR) -> bool:
        color = self.get_color()
        has_hole = color == hole_color
        if os.getenv("DEBUG"):
            self.debug_info.send(
                f"Buraco {'' if has_hole else 'não '}reconhecido.",
                System.color_sensor_detections,
            )
        return has_hole


class Motor(Device):
    def __init__(
        self,
        robot,
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
        if os.getenv("DEBUG"):
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

        if os.getenv("DEBUG"):
            self.debug_info.send(
                f"=== Começando a girar {turn_angle} rad para {direction}, "
                f"com a correção de {self._get_angle_imprecision(direction)} "
                "rad ===",
                System.motor_rotation,
            )

        turn_angle -= self._get_angle_imprecision(direction)
        self._set_angle_imprecision(0, direction)

        angle_accumulated_delta = 0
        rotation_angle = imu.get_rotation_angle()

        while self._robot.step(self._time_step) != -1:
            new_robot_angle = imu.get_rotation_angle()
            angle_accumulated_delta += IMU.get_delta_rotation(
                rotation_angle, new_robot_angle
            )
            angle_to_rotate = turn_angle - angle_accumulated_delta

            if os.getenv("DEBUG"):
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
            if os.getenv("DEBUG"):
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

                if os.getenv("DEBUG"):
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
    ) -> MovementConsequence:
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
        :return: If the robot was able to move. False if

        OBS:
        - Moves at `high_speed`, until it lasts `slow_down_dist` to move, when
        it slows down to `slow_down_speed`. This approach is intended to make
        the movement quick in general, but with a good precision as it moves
        slower in the end of the movement.
        - Holes are not detected when moving backward.
        """
        initial_position = gps.get_coordinates()

        if os.getenv("DEBUG"):
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

            if os.getenv("DEBUG"):
                self.debug_info.send(
                    f"Já moveu {traversed_dist}, sendo {x_delta=} e {y_delta=}.",
                    System.motor_movement,
                )

            left_velocity, right_velocity = Motor.movement_velocity_controller(
                dist - traversed_dist,
                direction,
                rotation_angle_error,
                slow_down_dist,
                high_speed,
                slow_down_speed,
            )
            self._left_motor.setVelocity(left_velocity)
            self._right_motor.setVelocity(right_velocity)

            if os.getenv("DEBUG"):
                self.debug_info.send(
                    f"Definindo velocidades como: esquerda={left_velocity} e "
                    f"direita={right_velocity}",
                    System.motor_velocity,
                )

            if traversed_dist >= dist:
                self.stop()

                self._set_move_imprecision(traversed_dist - dist, direction)

                if os.getenv("DEBUG"):
                    self.debug_info.send(
                        f"Fim do movimento, andou {traversed_dist} do "
                        f"objetivo: {dist} para {direction}. Passou em "
                        f"{self._get_move_imprecision(direction)} como "
                        "imprecisão para ser corrigida depois.",
                        System.motor_movement,
                    )

                break

            if lidar.wall_collision("front" if direction == "forward" else "back"):
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
                )

                if os.getenv("DEBUG"):
                    self.debug_info.send(
                        "Retornou à posição antiga após colidir com parede.",
                        System.lidar_wall_detection,
                    )

                raise WallColisionError()

            if color_sensor.has_hole():
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
                )

                if os.getenv("DEBUG"):
                    self.debug_info.send(
                        "Retornou à posição antiga após achar buraco.",
                        System.lidar_wall_detection,
                    )

                return MovementConsequence.hole
        return MovementConsequence.moved

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

    @staticmethod
    def movement_velocity_controller(
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

        if os.getenv("DEBUG") and abs(rotation_angle_error) > MAX_SPEED:
            logger.warning(
                "No PID, o erro para ser corrigido da rotação do "
                f"robô é: {rotation_angle_error}, maior do que a "
                f"maior velocidade do robô: {MAX_SPEED}"
            )

        speed = high_speed if dist_to_move > slow_down_dist else low_speed
        left_velocity = speed - rotation_angle_error
        right_velocity = speed + rotation_angle_error

        if direction == "backward":
            left_velocity *= -1
            right_velocity *= -1

        return left_velocity, right_velocity


# Initialize DebugInfo instance
try:
    debug_info = DebugInfo(
        [
            e
            for e in ALL_SYSTEMS
            if e
            not in [
                System.lidar_measures,
                System.lidar_general,
            ]
        ]
    )
except Exception:
    if os.getenv("DEBUG"):
        logger.error("Erro ao inicializar o logger", exc_info=True)
        raise


# Initialize robot and devices
try:
    robot = Robot()

    motor = Motor(robot, debug_info)
    lidar = Lidar(robot, debug_info)
    gps = GPS(robot, debug_info)
    imu = IMU(robot, debug_info)
    color_sensor = ColorSensor(robot, debug_info)
except Exception:
    if os.getenv("DEBUG"):
        logger.error("Erro durante inicialização do robô", exc_info=True)
        raise


# Show Robot attributes of initialization
print("========= Informações do robô inicializado ==========")
if os.getenv("DEBUG"):
    print("Modo do robô: teste")
else:
    print("Modo do robô: competição")

if os.getenv("DEBUG"):
    debug_info.send("\nSobre o LIDAR", System.lidar_general)
    debug_info.send(
        f"\tA distância percebida é [{lidar.min_range};{lidar.max_range}]",
        System.lidar_general,
    )
    debug_info.send(
        f"\tHá {lidar.horizontal_resolution} medições na horizontal "
        f"e {lidar.number_of_layers} layers na vertical",
        System.lidar_general,
    )
    debug_info.send(
        f"\tO FOV, entre 0 e 2*PI é: {lidar.field_of_view}",
        System.lidar_general,
    )
print("=====================================================\n")


# Robot main/loop routine
while robot.step(int(os.getenv("TIME_STEP", 32))) != -1:
    try:
        # TODO => main code
        pass
        motor.move("forward", gps, lidar, color_sensor, TILE_SIZE * 2)
        import time

        time.sleep(5)

    except Exception:
        if os.getenv("DEBUG"):
            logging.error("Erro na rotina principal do robô", exc_info=True)
            raise
