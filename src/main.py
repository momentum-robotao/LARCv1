import logging
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from functools import wraps
from typing import Any, Callable, Literal

from controller import Robot as WebotsRobot  # type: ignore

DEBUG = (os.getenv("DEBUG", "") + " ").upper()[0] in ["T", "1"]

PI = 3.14159265359
DEGREE_IN_RAD = 0.0174533

SLOW_DOWN_SPEED = 0.1
SLOW_DOWN_DIST = 0.001
MAX_SPEED = 6.28
KP = 1.0
TILE_SIZE = 0.12
WALL_TICKNESS = 0.01
ROBOT_RADIUS = 0.0355
MAX_WALL_DISTANCE = 0.030
EXPECTED_WALL_DISTANCE = 0.019
WALL_COLLISION_DISTANCE = 0.002

# TODO: adjust these constants
ORTOGONAL_MAX_DIST_IF_WALL = TILE_SIZE / 2
DIAGONAL_MAX_DIST_IF_WALL1 = (
    0.078380665  # 0.5*TILE_SIZE*cos 25°*(1+((1-tg 25)/(2*tg 25)))
)
DIAGONAL_MAX_DIST_IF_WALL2 = (
    0.135481996  # 0.5*TILE_SIZE*cos 25°*(1+3*((1-tg 25)/(2*tg 25)))
)


HOLE_COLOR = b"...\xff"

AreaDFSMappable = Literal[1, 2]
Side = Literal["front", "back", "left", "right"]
Quadrant = Literal["top_left", "top_right", "bottom_left", "bottom_right"]
RobotQuadrant = Literal[
    "front_left", "front_right", "back_left", "back_right", "front_center"
]
Numeric = float | int

CENTRAL_ANGLE_OF_SIDE: dict[Side, Numeric] = {
    "front": 0,
    "right": 90,
    "back": 180,
    "left": 270,
}

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
    logger.info(f"Criado log com sucesso em: {os.getenv('LOG_PATH')}")
except Exception:
    if DEBUG:
        logging.error("Erro ao inicializar o logger", exc_info=True)
        raise


class WallColisionError(Exception):
    """
    Exception raised when the robot colides with a wall and it
    has already returned to its last position.
    """

    pass


class MovementResult(Enum):
    moved = "moved"
    left_hole = "left hole"
    right_hole = "right hole"
    left_right_hole = "left right hole"


class System(Enum):
    initialization = "initialization"
    unknown_error = "unknown error"
    delay = "delay"
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
    maze_snapshot = "maze_snapshot"
    maze_changes = "maze changes"
    dfs_state = "dfs state"
    dfs_verification = "dfs verification"
    dfs_decision = "dfs decision"
    debug_info = "debug info"


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


WallToken = HazmatSign | Victim


class SpecialTileType(Enum):
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
    walls: set[Side] = field(default_factory=set)


@dataclass
class Tile:
    quadrants: dict[Quadrant, QuarterTile] = field(default_factory=dict)
    special_type: SpecialTileType | None = None


class Coordinate:
    def __init__(self, x: Numeric, y: Numeric):
        self.x = x
        self.y = y

    def __hash__(self) -> int:
        return hash(f"x={self.x}; y={self.y}")

    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Coordinate):
            if DEBUG:
                return NotImplemented
            return False

        return self.x == value.x and self.y == value.y

    def __repr__(self) -> str:
        return f"Coordinate(x={self.x}, y={self.y})"

    def __add__(self, delta: "Coordinate") -> "Coordinate":
        return Coordinate(self.x + delta.x, self.y + delta.y)

    def __sub__(self, delta: "Coordinate") -> "Coordinate":
        return Coordinate(self.x - delta.x, self.y - delta.y)

    def __mul__(self, multiplier: Numeric) -> "Coordinate":
        return Coordinate(self.x * multiplier, self.y * multiplier)


@dataclass
class RGB:
    red: float
    green: float
    blue: float


class DebugInfo:
    def __init__(
        self,
        systems_to_debug: list[System] | None = None,
        systems_to_ignore: list[System] | None = None,
    ) -> None:
        self.systems_to_debug = systems_to_debug or []
        self.systems_to_ignore = systems_to_ignore or []
        if not DEBUG:
            self.systems_to_debug = []
        self.send(
            "Inicializado `DebugInfo`.\n"
            f"Debugar: {self.systems_to_debug}\nIgnorar: {self.systems_to_ignore}",
            System.debug_info,
        )

    def send(
        self,
        message: str,
        system_being_debugged: System,
        level: Literal["info", "error", "debug", "warning", "critical"] = "info",
    ) -> None:
        if system_being_debugged not in self.systems_to_debug:
            if DEBUG and system_being_debugged not in self.systems_to_ignore:
                logger.debug(f"{system_being_debugged} => {message}")
            return
        if level in ["error", "critical"]:
            getattr(logger, level)(
                f"{system_being_debugged} => {message}", exc_info=True
            )
        else:
            getattr(logger, level)(f"{system_being_debugged} => {message}")
        print(f"{level.upper()} {system_being_debugged} => {message}")


def round_if_almost_0(value: float):
    return 0 if round(value, 2) == 0 else value


def delay(
    robot,
    debug_info: DebugInfo,
    time_ms: Numeric,
    time_step: int = int(os.getenv("TIME_STEP", 32)),
):
    if DEBUG:
        debug_info.send(f"Esperando: {time_ms}ms", System.delay)

    init_time = robot.getTime()
    while robot.step(time_step) != -1:
        time_elapsed_ms = (robot.getTime() - init_time) * 1000.0
        if time_elapsed_ms > time_ms:
            break


def cyclic_angle(angle: Numeric) -> Numeric:
    """
    It may be used for cases in which angles are cyclic and should
    be considered in range[0; 2*PI].
    Example of returns depending on the `angle`:
    - `PI` => returns `PI`
    - `-0.5 * PI` => returns `1.5 * PI`, because it is turning clockwise
    this angle, so it is 0.5 * PI before a complete turn (2 * PI), then
    it is `2.0 * PI - 0.5 * PI = 1.5 * PI`
    - `5.5 * PI` => returns `1.5 * PI`, because 3.5 * PI, is 2 complete
    turns + 1.5 * PI

    :return: The `angle` in range [0; 2*PI].
    :rtype: Same type as `angle`.
    """
    while angle < 0:
        angle += 2 * PI
    while angle >= 2 * PI:
        angle -= 2 * PI

    return angle


# For each wall, sorted by distance, that robot may collide after moving with some rotation angle:
# Coordinate delta from robot position to quarter tile and side of it that contains the wall
DEGREE_WALL_FROM_ROBOT_POSITION: dict[
    RobotQuadrant, dict[int, list[tuple[Coordinate, Side]]]
] = {
    "front_left": {
        0: [(Coordinate(0, 1), "front")],
        45: [(Coordinate(1, 1), "front"), (Coordinate(1, 2), "right")],
        90: [(Coordinate(1, 1), "right")],
        135: [(Coordinate(1, 0), "right"), (Coordinate(2, 0), "back")],
        180: [(Coordinate(1, 0), "back")],
        225: [(Coordinate(0, 0), "back"), (Coordinate(0, -1), "left")],
        270: [(Coordinate(0, 0), "left")],
        315: [(Coordinate(0, 1), "left"), (Coordinate(-1, 1), "front")],
        360: [(Coordinate(0, 1), "front")],
    },
    "front_right": {
        0: [(Coordinate(1, 1), "front")],
        45: [(Coordinate(1, 1), "right"), (Coordinate(2, 1), "front")],
        90: [(Coordinate(1, 0), "right")],
        135: [(Coordinate(1, 0), "back"), (Coordinate(1, -1), "right")],
        180: [(Coordinate(0, 0), "back")],
        225: [(Coordinate(0, 0), "left"), (Coordinate(-1, 0), "back")],
        270: [(Coordinate(0, 1), "left")],
        315: [(Coordinate(0, 1), "front"), (Coordinate(0, 2), "left")],
        360: [(Coordinate(1, 1), "front")],
    },
    "front_center": {
        0: [(Coordinate(0, 2), "right")],
        90: [(Coordinate(2, 0), "front")],
        180: [(Coordinate(0, -1), "right")],
        270: [(Coordinate(-1, 0), "front")],
        360: [(Coordinate(0, 2), "right")],
    },
}

WALL_FROM_ROBOT_POSITION: dict[
    RobotQuadrant, dict[float, list[tuple[Coordinate, Side]]]
] = {
    robot_quadrant: {
        round(degree_angle * DEGREE_IN_RAD, 2): wall_deltas
        for degree_angle, wall_deltas in DEGREE_WALL_FROM_ROBOT_POSITION[
            robot_quadrant
        ].items()
    }
    for robot_quadrant in DEGREE_WALL_FROM_ROBOT_POSITION
}


DEGREE_TARGET_COORDINATE = {
    0: Coordinate(0, 1),
    45: Coordinate(1, 1),
    90: Coordinate(1, 0),
    135: Coordinate(1, -1),
    180: Coordinate(0, -1),
    225: Coordinate(-1, -1),
    270: Coordinate(-1, 0),
    315: Coordinate(-1, 1),
    360: Coordinate(0, 1),
}

TARGET_COORDINATE = {
    round(degree_angle * DEGREE_IN_RAD, 2): coordinate
    for degree_angle, coordinate in DEGREE_TARGET_COORDINATE.items()
}

DELTA_TO_QUADRANT: dict[Coordinate, Quadrant] = {
    Coordinate(0, 0): "bottom_left",
    Coordinate(1, 0): "bottom_right",
    Coordinate(0, 1): "top_left",
    Coordinate(1, 1): "top_right",
}


# TODO: rename RobotQuadrant as it has "front_center" now
# TODO: merge with `coordinate_after_move` to get for example the most similar rotation angle
def calculate_wall_position(
    robot_position: Coordinate,
    robot_quadrant: RobotQuadrant,
    angle: Numeric,
    wall_idx: int,
    angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2)),
) -> tuple[Coordinate, Side]:
    equivalent_rotation_angle = None
    equivalent_rotation_angle = round(angle, 2)
    for testing_angle in WALL_FROM_ROBOT_POSITION[robot_quadrant]:
        if abs(testing_angle - equivalent_rotation_angle) <= angle_max_difference:
            equivalent_rotation_angle = testing_angle

    if equivalent_rotation_angle is None:
        if DEBUG:
            error_message = (
                f"Inválido {equivalent_rotation_angle=} ao tentar calcular posição da parede "
                f"detectada de {robot_position=} olhando pelo {robot_quadrant=}"
            )
            logging.error(error_message)
            logging.debug(
                f"Ângulos disponíveis: {list(WALL_FROM_ROBOT_POSITION[robot_quadrant].keys())}"
            )
            raise ValueError(error_message)

        # Gets the most similar angle categorized
        equivalent_rotation_angle = equivalent_rotation_angle
        for testing_angle in WALL_FROM_ROBOT_POSITION[robot_quadrant]:
            if (
                abs(testing_angle - equivalent_rotation_angle)
                < equivalent_rotation_angle
            ):
                equivalent_rotation_angle = testing_angle

    delta_from_robot_position, side = WALL_FROM_ROBOT_POSITION[robot_quadrant][
        equivalent_rotation_angle
    ][wall_idx]
    quarter_tile = robot_position + delta_from_robot_position
    return quarter_tile, side


def coordinate_after_move(
    position: Coordinate,
    angle: Numeric,
    angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2)),
) -> Coordinate:
    angle = round(angle, 2)
    for target_angle in TARGET_COORDINATE:
        if abs(target_angle - angle) <= angle_max_difference:
            return position + TARGET_COORDINATE[target_angle]

    if DEBUG:
        error_message = (
            f"Inválido {angle=} ao tentar pegar coordenada "
            f"após movimentação começando de {position}"
        )
        logging.error(error_message)
        logging.debug(f"Ângulos disponíveis: {list(TARGET_COORDINATE.keys())}")
        raise ValueError(error_message)

    # Gets the most similar angle categorized
    most_similar_angle = (angle, position + Coordinate(0, 1))
    for reference_angle, delta in TARGET_COORDINATE.items():
        if abs(reference_angle - angle) < most_similar_angle[0]:
            most_similar_angle = (reference_angle, position + delta)

    return most_similar_angle[1]


def tile_pos_with_quarter_tile(quarter_tile_pos: Coordinate) -> Coordinate:
    """
    :return: The `Coordinate` of tile that contains this quarter tile.
    """
    return Coordinate(quarter_tile_pos.x // 2, quarter_tile_pos.y // 2)


def quarter_tile_quadrant(quarter_tile_pos: Coordinate) -> Quadrant:
    """
    :return: The quadrant that the quarter tile is corresponding
    to the tile in which it is located.
    """
    delta = quarter_tile_pos - tile_pos_with_quarter_tile(quarter_tile_pos) * 2
    return DELTA_TO_QUADRANT[delta]


def side_angle_from_map_angle(map_angle: float, robot_rotation_angle: float) -> float:
    """
    Returns the angle of the side of the robot that is in the same direction
    of the `map_angle`.
    """
    return cyclic_angle(map_angle - robot_rotation_angle)


def get_blocking_wall(wall_distance: float, delta_angle_in_degree: int) -> int:
    if delta_angle_in_degree in [90, 0, -90]:
        wall_blocking = 0 if wall_distance <= ORTOGONAL_MAX_DIST_IF_WALL else -1
    elif delta_angle_in_degree in [45, -45]:
        wall_blocking = (
            0
            if wall_distance <= DIAGONAL_MAX_DIST_IF_WALL1
            else (1 if wall_distance <= DIAGONAL_MAX_DIST_IF_WALL2 else -1)
        )
    return wall_blocking


def get_central_blocking_wall(wall_distance: float, delta_angle_in_degree: int) -> int:
    if delta_angle_in_degree not in [-90, 0, 90]:
        return -1
    return 0 if wall_distance <= ORTOGONAL_MAX_DIST_IF_WALL else -1


def log_map_change(func: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(func)
    def wrapper(self: "Maze", *args, **kwargs):
        map_before = str(self)
        result = func(self, *args, **kwargs)
        map_after = str(self)

        if DEBUG and map_before != map_after:
            self.debug_info.send(
                f"Mapa modificado para: {map_after}",
                System.maze_snapshot,
            )
        return result

    return wrapper


ObjectsMaze = dict[Numeric, dict[Numeric, Tile]]
AnswerMaze = list[list[str]]


class Maze:
    """
    OBS: note that `ObjectsMaze` are composed by full tile coordinates
    while methods from `Maze` deals with quarter-tile coordinates. This
    logic is internal to the class.
    """

    def __init__(self, debug_info: DebugInfo) -> None:
        self.objects: dict[Numeric, dict[Numeric, Tile]] = dict()
        self.wall_tokens: list[tuple[Coordinate, Side]] = []
        self.debug_info = debug_info

    @staticmethod
    def check_position(quarter_tile_pos: Coordinate) -> Coordinate:
        if not isinstance(quarter_tile_pos.x, int) or not isinstance(
            quarter_tile_pos.y, int
        ):
            if DEBUG:
                raise ValueError("Coordenada para mapeamento não é inteira")
            else:
                quarter_tile_pos = Coordinate(
                    round(quarter_tile_pos.x), round(quarter_tile_pos.y)
                )

        return quarter_tile_pos

    def __str__(self) -> str:
        map_str = ""
        for x in self.objects:
            for y in self.objects[x]:
                tile = self.objects[x][y]
                map_str += f"({x},{y})={tile}, "
            map_str += "\n"
        return map_str

    def mark_quarter_tile(self, quarter_tile_pos: Coordinate) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)

        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)
        self.objects.setdefault(tile_pos.x, {}).setdefault(tile_pos.y, Tile())
        self.objects[tile_pos.x][tile_pos.y].quadrants.setdefault(
            quarter_tile_quadrant(quarter_tile_pos), QuarterTile()
        )

    def is_visited(self, quarter_tile_pos: Coordinate) -> bool:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        return quarter_tile_pos.y in self.objects.get(quarter_tile_pos.x, {})

    @log_map_change
    def add_wall(self, quarter_tile_pos: Coordinate, side: Side) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        self.mark_quarter_tile(quarter_tile_pos)

        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)
        self.objects[tile_pos.x][tile_pos.y].quadrants[
            quarter_tile_quadrant(quarter_tile_pos)
        ].walls.add(side)

    @log_map_change
    def set_tile_type(
        self,
        quarter_tile_pos: Coordinate,
        tile_type: SpecialTileType,
    ) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)

        old_tile_type = self.objects[tile_pos.x][tile_pos.y].special_type
        if old_tile_type is None or old_tile_type != tile_type:
            if DEBUG:
                self.debug_info.send(
                    f"Tipo do tile que contém {quarter_tile_pos} "
                    f"era {old_tile_type} e virou {tile_type}",
                    System.maze_changes,
                    level="error",
                )
                raise ValueError(
                    f"Tipo do tile que contém {quarter_tile_pos} foi mudado."
                )

        self.objects[tile_pos.x][tile_pos.y].special_type = tile_type

    def add_wall_token(self, quarter_tile_pos: Coordinate, side: Side) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        self.wall_tokens.append((quarter_tile_pos, side))

    def get_answer_map(self) -> AnswerMaze:
        answer_map = AnswerMaze()
        return answer_map


class Device(ABC):
    """
    Classes responsible for interacting with the robot. It includes classes
    that get data from robot (they are sensors) and classes that execute
    actions on the robot (they are actuators).
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

    def show_initialization_information(self) -> None:
        self.debug_info.send("\nSobre o LIDAR", System.initialization)
        self.debug_info.send(
            f"\tA distância percebida é [{self.min_range};{self.max_range}]",
            System.initialization,
        )
        self.debug_info.send(
            f"\tHá {self.horizontal_resolution} medições na horizontal "
            f"e {self.number_of_layers} layers na vertical",
            System.initialization,
        )
        self.debug_info.send(
            f"\tO FOV, entre 0 e 2*PI é: {self.field_of_view}",
            System.initialization,
        )

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

        if DEBUG:
            self.debug_info.send(f"{distances=}", System.lidar_measures)
        return distances

    def get_distances_by_angle(self) -> dict[float, float]:
        """
        :return: A dict where `dict[angle] = distance measured in this angle`.
        """
        distances = self.get_distances()
        distances_by_angle = dict()
        for measure_idx, dist in enumerate(distances):
            angle = round(self._get_measure_angle(measure_idx), 2)
            distances_by_angle[angle] = dist

        if DEBUG:
            self.debug_info.send(
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

        if DEBUG:
            self.debug_info.send(
                f"Pegas medições do intervalo cíclico baseado nos ângulos: "
                f"[{initial_angle};{end_angle}]. Medições: {distances_of_range}",
                System.lidar_range_measures,
            )

        return distances_of_range

    def get_side_distance(
        self,
        side: Side | Numeric,
        field_of_view: float = 10 * DEGREE_IN_RAD,
        remove_inf_measures: bool = True,
    ) -> float:
        """
        :param side: If it is a `Side`, the measures are centralized in
        this side of the robot, but if it is a `float` or `int` it is
        centralized on this angle, that should be from [0;2*PI] (in rad).

        :return: The average distance from `field_of_view` centralized in
        a side of the robot. Excluding INF values.
        """
        if isinstance(side, str):
            central_angle = CENTRAL_ANGLE_OF_SIDE[side]
            start_angle = central_angle * DEGREE_IN_RAD - field_of_view / 2
            end_angle = central_angle * DEGREE_IN_RAD + field_of_view / 2
        elif isinstance(side, (int, float)):
            start_angle = side - field_of_view / 2
            end_angle = side + field_of_view / 2

        distances = self.get_distances_of_range(
            cyclic_angle(start_angle), cyclic_angle(end_angle)
        )
        if min(distances) != float("inf") and remove_inf_measures:
            distances = [dist for dist in distances if dist != float("inf")]

        average_distance = sum(distances) / len(distances)

        if DEBUG:
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

        if DEBUG:
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
        if DEBUG:
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
        if self.has_wall("left"):
            rotation_angle_error = (
                self.get_side_distance("left") - expected_wall_distance
            ) * kp
        if self.has_wall("right"):
            rotation_angle_error = (
                (self.get_side_distance("right") - expected_wall_distance) * kp * -1
            )

        if DEBUG:
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
        x, _z, y = positions
        coordinate = Coordinate(x, y)
        if DEBUG:
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
        if DEBUG:
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
        if DEBUG:
            self.debug_info.send(
                f"Cor RGB reconhecida: {color}", System.color_sensor_measures
            )
        return color

    def has_hole(self, hole_color: bytes = HOLE_COLOR) -> bool:
        color = self.get_color()
        has_hole = color == hole_color
        if DEBUG:
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


class Robot:
    def __init__(
        self,
        webots_robot: WebotsRobot,
        motor: Motor,
        lidar: Lidar,
        gps: GPS,
        imu: IMU,
        color_sensor: ColorSensor,
        debug_info: DebugInfo,
        time_step: int = int(os.getenv("TIME_STEP", 32)),
    ):
        self.webots_robot = webots_robot

        self.motor = motor
        self.lidar = lidar
        self.gps = gps
        self.imu = imu
        self.color_sensor = color_sensor

        self.time_step = time_step
        self.debug_info = debug_info

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


def adjust_wall_distance(
    robot: Robot, angle_max_difference=float(os.getenv("ANGLE_MAX_DIFFERENCE", 0.2))
) -> None:
    # TODO: ensure that robot initial angle (that we use as reference to angle 0)
    # is perpendicular to wall, otherwise multiples of 90 could not be the ones that
    # indicate that we are perpendicular to the wall and 45 + 90*x could indicate it
    # perpendicular_to_wall = False
    # for rotation_angle_perpendicular_to_wall in [0, 90, 180, 270, 360]:
    #     expected_angle = rotation_angle_perpendicular_to_wall * DEGREE_IN_RAD
    #     if abs(robot.imu.get_rotation_angle() - expected_angle) <= angle_max_difference:
    #         perpendicular_to_wall = True
    # if not perpendicular_to_wall:
    #     continue
    return


def dfs(
    position: Coordinate,
    maze: Maze,
    robot: Robot,
    debug_info: DebugInfo,
    area: AreaDFSMappable,
):
    """
    Map all the specified `area`, then go to transition `area+1`.
    Robot position is specified by the lower left coordinate.

    :return: The final position of the robot.
    """
    debug_info.send(f"Começando DFS em {position=} da {area=}", System.dfs_state)
    adjust_wall_distance(robot)

    robot.step()
    start_angle = robot.imu.get_rotation_angle()
    debug_info.send(
        f"DFS de {position=} começou com {start_angle=}rad", System.dfs_verification
    )

    # Transition to neighbours on grid, prioritizing front, left and right
    # before diagonals
    for delta_angle_in_degree in [0, -90, 90, -45, 45]:
        delta_angle = delta_angle_in_degree * DEGREE_IN_RAD

        movement_angle = cyclic_angle(start_angle + delta_angle)
        side_angle = side_angle_from_map_angle(
            movement_angle, robot.imu.get_rotation_angle()
        )
        new_robot_position = coordinate_after_move(position, movement_angle)

        debug_info.send(
            f"DFS de {position=}, olhando vizinho de ângulo:\n"
            f"- {delta_angle_in_degree}° em relação ao ângulo inicial da DFS\n"
            f"- {movement_angle}rad de direção em relação ao mapa\n"
            f"- {side_angle}rad em relação à frente do robô.\n"
            f"Esse vizinho está na coordenada {new_robot_position}",
            System.dfs_state,
        )

        # For each side of the tiles that would be used to go to new position,
        # map if there is a wall there or not
        left_wall_angle = cyclic_angle(side_angle - 20 * DEGREE_IN_RAD)
        right_wall_angle = cyclic_angle(side_angle + 20 * DEGREE_IN_RAD)

        left_wall_distance = robot.lidar.get_side_distance(left_wall_angle)
        right_wall_distance = robot.lidar.get_side_distance(right_wall_angle)
        central_wall_distance = robot.lidar.get_side_distance(side_angle)

        debug_info.send(
            f"Distância das paredes no caminho para essa nova posição: {left_wall_distance=}m, "
            f"{right_wall_distance=}m e {central_wall_distance=}m",
            System.dfs_verification,
        )

        left_wall_blocking = get_blocking_wall(
            left_wall_distance, delta_angle_in_degree
        )
        right_wall_blocking = get_blocking_wall(
            right_wall_distance, delta_angle_in_degree
        )
        central_wall_blocking = get_central_blocking_wall(
            central_wall_distance, delta_angle_in_degree
        )

        # blocked? don't move and map these walls
        debug_info.send(
            f"Paredes detectadas: {left_wall_blocking=}; {right_wall_blocking=} "
            f"e {central_wall_blocking=}",
            System.dfs_verification,
        )
        if left_wall_blocking != -1:
            quarter_tile, side = calculate_wall_position(
                position, "front_left", movement_angle, left_wall_blocking
            )
            maze.add_wall(quarter_tile, side)
        if right_wall_blocking != -1:
            quarter_tile, side = calculate_wall_position(
                position, "front_right", movement_angle, right_wall_blocking
            )
            maze.add_wall(quarter_tile, side)
        if left_wall_blocking != -1 or right_wall_blocking != -1:
            debug_info.send(
                "Há paredes no caminho para o vizinho: não será visitado",
                System.dfs_decision,
            )
            continue

        if (
            central_wall_blocking != -1
            and right_wall_blocking == -1
            and left_wall_blocking == -1
        ):
            quarter_tile, side = calculate_wall_position(
                position, "front_center", movement_angle, central_wall_blocking
            )
            maze.add_wall(quarter_tile, side)
            debug_info.send(
                "Há parede central no caminho para o vizinho: não será visitado",
                System.dfs_decision,
            )
            continue

        # visited? don't move
        if all(
            maze.is_visited(new_robot_position + delta)
            for delta in [
                Coordinate(0, 0),
                Coordinate(0, 1),
                Coordinate(1, 0),
                Coordinate(1, 1),
            ]
        ):
            debug_info.send(
                "Vizinho já foi visitado: não será visitado novamente",
                System.dfs_decision,
            )
            continue

        # otherwise, visit it recursively (with dfs)
        debug_info.send("Movendo para o vizinho", System.dfs_decision)
        new_position_distance = (
            TILE_SIZE / 2 * (1.44 if delta_angle_in_degree in [45, -45] else 1)
        )
        robot.motor.rotate_to_angle(movement_angle, robot.imu)
        try:
            movement_result = robot.motor.move(
                "forward",
                robot.gps,
                robot.lidar,
                robot.color_sensor,
                new_position_distance,
            )  # TODO: checar se buraco é de um dos lados apenas
            if movement_result == MovementResult.left_right_hole:
                continue  # TODO: mapear e logar
            elif MovementResult.moved:
                pass
            else:
                continue  # TODO: Logar esse problema
        except WallColisionError:
            continue  # TODO: mapear e logar

        dfs(new_robot_position, maze, robot, debug_info, area)

        debug_info.send("Retornando do vizinho", System.dfs_decision)
        robot.motor.move(
            "backward",
            robot.gps,
            robot.lidar,
            robot.color_sensor,
            new_position_distance,
        )

        """
        TODO: use bfs to return to the last tile with unvisited neighbours
        that you are able to go to it.
        Return from dfs the actual position so when needed, bfs is called to
        comeback
        First version: just comeback from the tile after calling another dfs
        OBS: rotation angle may be totally different, take care of it
        """
    debug_info.send(
        f"Finalizando DFS de {position=}. Voltando para {start_angle=}",
        System.dfs_decision,
    )
    robot.motor.rotate_to_angle(start_angle, robot.imu)


def solve_map(robot: Robot, debug_info: DebugInfo) -> None:
    maze = Maze(debug_info)

    # TODO: map initial positions of robot as the start tile
    """
    Coordinate(0, 1),
    Coordinate(1, 1),
    Coordinate(0, 0),
    Coordinate(1, 0),
    """
    ...
    position = Coordinate(0, 0)
    position = dfs(position, maze, robot, debug_info, area=1)
    # TODO: transition between maps
    # position = dfs(position, maze, all_robot_info, area=2)


def main() -> None:
    # Initialize DebugInfo instance
    if DEBUG:
        logger.info(f"Começando nova execução: {datetime.now()}")
    try:
        # debug_info = DebugInfo(
        #     systems_to_debug=[
        #         e for e in ALL_SYSTEMS if str(e) not in [str(System.lidar_measures)]
        #     ],
        #     systems_to_ignore=[System.lidar_measures],
        # )
        want = [System.dfs_state]
        debug_info = DebugInfo(
            systems_to_debug=want,
            systems_to_ignore=[
                e for e in ALL_SYSTEMS if str(e) not in [str(w) for w in want]
            ],
        )
    except Exception:
        if DEBUG:
            logger.error("Erro ao inicializar o debug info", exc_info=True)
            raise

    # Initialize robot and devices
    try:
        webots_robot = WebotsRobot()
        webots_robot.step(int(os.getenv("TIME_STEP", 32)))

        motor = Motor(webots_robot, debug_info)
        lidar = Lidar(webots_robot, debug_info)
        gps = GPS(webots_robot, debug_info)
        imu = IMU(webots_robot, debug_info)
        color_sensor = ColorSensor(webots_robot, debug_info)

        robot = Robot(webots_robot, motor, lidar, gps, imu, color_sensor, debug_info)
    except Exception:
        if DEBUG:
            debug_info.send(
                "Erro durante inicialização do robô", System.initialization, "error"
            )
            raise

    # Solve map
    try:
        # while robot.step() != -1:
        #     print(robot.imu.get_rotation_angle())
        solve_map(robot, debug_info)
    except Exception:
        if DEBUG:
            debug_info.send(
                "Erro inesperado enquanto resolvia o mapa",
                System.unknown_error,
                "critical",
            )
            raise


main()
