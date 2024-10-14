import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Literal

DEBUG = (os.getenv("DEBUG", "") + " ").upper()[0] in ["T", "1"]
ON_DOCKER = ((os.getenv("ON_DOCKER", "") + " ").upper()[0] in ["T", "1"]) and DEBUG

PI = 3.14159265359
DEGREE_IN_RAD = 0.0174533

METER_TO_CM = 100

SLOW_DOWN_SPEED = 0.1
SLOW_DOWN_DIST = 0.001
MAX_SPEED = 6.28
KP = 5.0
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

if ON_DOCKER:
    NGROK_URL = ""
    with open("./ngrok.txt", "r") as file:
        NGROK_URL = file.readlines()[0]

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


class MovementResult(Enum):
    moved = "moved"
    left_hole = "left hole"
    right_hole = "right hole"
    left_right_hole = "left right hole"


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
