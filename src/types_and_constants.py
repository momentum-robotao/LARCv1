import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Literal


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


DEBUG = (os.getenv("DEBUG", "") + " ").upper()[0] in ["T", "1"]

PI = 3.14159265359
DEGREE_IN_RAD = 0.0174533

METER_TO_CM = 100

MAX_SPEED = 6.28

# Move constants
SLOW_DOWN_SPEED = MAX_SPEED / 2
SLOW_DOWN_DIST = 0.005
# Rotate constants
SLOW_DOWN_ANGLE = 0.1
SLOW_DOWN_ROTATE_SPEED = MAX_SPEED / 10

KP = 0.0
TILE_SIZE = 0.12
WALL_TICKNESS = 0.01
ROBOT_RADIUS = 0.0355
MAX_WALL_DISTANCE = 0.030
EXPECTED_WALL_DISTANCE = 0.0198
WALL_COLLISION_DISTANCE = 0.002
DIST_BEFORE_HOLE = 0.02
THRESHOLD_SLAM = ROBOT_RADIUS/2
SLAM_MAX_DISTANCE = 2*TILE_SIZE

ORTOGONAL_MAX_DIST_IF_WALL = TILE_SIZE / 2
DIAGONAL_MAX_DIST_IF_WALL1 = (
    0.078380665  # 0.5*TILE_SIZE*cos 25°*(1+((1-tg 25)/(2*tg 25)))
)
DIAGONAL_MAX_DIST_IF_WALL2 = (
    0.135481996  # 0.5*TILE_SIZE*cos 25°*(1+3*((1-tg 25)/(2*tg 25)))
)

AreaDFSMappable = Literal[1, 2]
Side = Literal["front", "back", "left", "right"]
Quadrant = Literal["top_left", "top_right", "bottom_left", "bottom_right"]
SideDirection = Literal["front_left", "front_right", "front_center"]
Numeric = float | int

CENTRAL_SIDE_ANGLE_OF_SIDE: dict[Side, Numeric] = {
    "front": 0,
    "right": 90,
    "back": 180,
    "left": 270,
}


class EndOfTimeError(Exception):
    """
    Exception raised when time is going to end.
    """


class WallColisionError(Exception):
    """
    Exception raised when the robot colides with a wall and it
    has already returned to its last position.
    """

    pass


class LackOfProgressError(Exception):
    """
    Exception raised when a lack of progress is detected.
    """

    pass


# Related to mapping
class MovementResult(Enum):
    moved = "moved"
    left_hole = "left hole"
    right_hole = "right hole"
    central_hole = "central hole"


class SpecialTileType(Enum):
    AREA_4 = "*"
    HOLE = "2"
    SWAMP = "3"
    CHECKPOINT = "4"
    STARTING = "5"
    PASSAGE_1_2 = "b"  # blue
    PASSAGE_1_3 = "y"  # yellow
    PASSAGE_1_4 = "g"  # green
    PASSAGE_2_3 = "p"  # purple
    PASSAGE_2_4 = "o"  # orange
    PASSAGE_3_4 = "r"  # red


class MappingEncode(Enum):
    """Encodings required by Erebus for objects."""

    WALL = "1"
    DEFAULT = "0"


@dataclass
class QuarterTile:
    walls: set[Side] = field(default_factory=set)


@dataclass
class Tile:
    quadrants: dict[Quadrant, QuarterTile] = field(default_factory=dict)
    special_type: SpecialTileType | None = None


# Data classes
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


@dataclass(frozen=True)
class RGB:
    red: float
    green: float
    blue: float

    def __eq__(self, other) -> bool:
        return (
            abs(self.red - other.red) <= 2
            and abs(self.green - other.green) <= 2
            and abs(self.blue - other.blue) <= 2
        )


# For each wall, sorted by distance, that robot may collide after moving with some rotation angle:
# Coordinate delta from robot position to quarter tile and side of it that contains the wall
DEGREE_WALL_FROM_ROBOT_POSITION: dict[
    SideDirection, dict[int, list[tuple[Coordinate, Side]]]
] = {
    "front_left": {
        0: [(Coordinate(0, -1), "front")],
        45: [(Coordinate(1, -1), "front"), (Coordinate(1, -2), "right")],
        90: [(Coordinate(1, -1), "right")],
        135: [(Coordinate(1, 0), "right"), (Coordinate(2, 0), "back")],
        180: [(Coordinate(1, 0), "back")],
        225: [(Coordinate(0, 0), "back"), (Coordinate(0, 1), "left")],
        270: [(Coordinate(0, 0), "left")],
        315: [(Coordinate(0, -1), "left"), (Coordinate(-1, -1), "front")],
        360: [(Coordinate(0, -1), "front")],
    },
    "front_right": {
        0: [(Coordinate(1, -1), "front")],
        45: [(Coordinate(1, -1), "right"), (Coordinate(2, -1), "front")],
        90: [(Coordinate(1, 0), "right")],
        135: [(Coordinate(1, 0), "back"), (Coordinate(1, 1), "right")],
        180: [(Coordinate(0, 0), "back")],
        225: [(Coordinate(0, 0), "left"), (Coordinate(-1, 0), "back")],
        270: [(Coordinate(0, -1), "left")],
        315: [(Coordinate(0, -1), "front"), (Coordinate(0, -2), "left")],
        360: [(Coordinate(1, -1), "front")],
    },
    "front_center": {
        0: [(Coordinate(0, -2), "right")],
        90: [(Coordinate(2, 0), "front")],
        180: [(Coordinate(0, 1), "right")],
        270: [(Coordinate(-1, 0), "front")],
        360: [(Coordinate(0, -2), "right")],
    },
}

WALL_FROM_ROBOT_POSITION: dict[
    SideDirection, dict[float, list[tuple[Coordinate, Side]]]
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
    0: Coordinate(0, -1),
    45: Coordinate(1, -1),
    90: Coordinate(1, 0),
    135: Coordinate(1, 1),
    180: Coordinate(0, 1),
    225: Coordinate(-1, 1),
    270: Coordinate(-1, 0),
    315: Coordinate(-1, -1),
    360: Coordinate(0, -1),
}

TARGET_COORDINATE_BY_MOVE_ANGLE = {
    round(degree_angle * DEGREE_IN_RAD, 2): coordinate
    for degree_angle, coordinate in DEGREE_TARGET_COORDINATE.items()
}

QUADRANT_OF_DELTA: dict[Coordinate, Quadrant] = {
    Coordinate(0, 0): "bottom_left",
    Coordinate(1, 0): "bottom_right",
    Coordinate(0, -1): "top_left",
    Coordinate(1, -1): "top_right",
}

ALL_QUADRANTS: list[Quadrant] = ["bottom_left", "bottom_right", "top_left", "top_right"]

ColoredSpecialTile = Literal[
    SpecialTileType.PASSAGE_1_2,
    SpecialTileType.PASSAGE_1_3,
    SpecialTileType.PASSAGE_1_4,
    SpecialTileType.PASSAGE_2_3,
    SpecialTileType.PASSAGE_2_4,
    SpecialTileType.PASSAGE_3_4,
    SpecialTileType.SWAMP,
    SpecialTileType.CHECKPOINT,
]


SPECIAL_TILE_COLOR_MAPPER: dict[RGB, ColoredSpecialTile] = {
    RGB(255, 255, 255): SpecialTileType.CHECKPOINT,
    RGB(80, 80, 255): SpecialTileType.PASSAGE_1_2,
    RGB(255, 255, 80): SpecialTileType.PASSAGE_1_3,
    RGB(255, 245, 80): SpecialTileType.PASSAGE_2_4,
    RGB(42, 254, 42): SpecialTileType.PASSAGE_1_4,
    RGB(175, 80, 245): SpecialTileType.PASSAGE_2_3,
    RGB(255, 79, 79): SpecialTileType.PASSAGE_3_4,
    RGB(235, 206, 126): SpecialTileType.SWAMP,
}
