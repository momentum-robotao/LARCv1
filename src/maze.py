from functools import wraps
from typing import Any, Callable

from debugging import DebugInfo, System
from helpers import quarter_tile_quadrant, tile_pos_with_quarter_tile
from types_and_constants import (
    DEBUG,
    DELTA_TO_QUADRANT,
    Coordinate,
    Numeric,
    QuarterTile,
    Side,
    SpecialTileType,
    Tile,
)

ObjectsMaze = dict[Numeric, dict[Numeric, Tile]]
AnswerMaze = list[list[str]]


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

    def _mark_quarter_tile(self, quarter_tile_pos: Coordinate) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)

        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)
        self.objects.setdefault(tile_pos.x, {}).setdefault(tile_pos.y, Tile())
        self.objects[tile_pos.x][tile_pos.y].quadrants.setdefault(
            quarter_tile_quadrant(quarter_tile_pos), QuarterTile()
        )

    def mark_visited(self, robot_position_quarter_tile: Coordinate) -> None:
        # TODO: make sure robot always have this 4 quarter tiles deltas => não
        # pode começar em posição que onde ele considera como +(0,1) por exemplo
        # é um com esses deltas e outros se ele andar pra frente
        for delta, _ in DELTA_TO_QUADRANT.items():
            self._mark_quarter_tile(robot_position_quarter_tile + delta)

    def is_visited(self, quarter_tile_pos: Coordinate) -> bool:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)

        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)
        quadrant = quarter_tile_quadrant(quarter_tile_pos)
        self.debug_info.send(
            f"Checking if tile {tile_pos} is visited in {quadrant=}.",
            System.maze_visited,
        )
        self.debug_info.send(f"Already visited in x={tile_pos.x}:", System.maze_visited)
        for y, visited_tile in self.objects.get(tile_pos.x, {}).items():
            self.debug_info.send(f" - {y=}: {visited_tile}", System.maze_visited)
        return (
            quadrant
            in self.objects.get(tile_pos.x, {}).get(tile_pos.y, Tile()).quadrants
        )

    @log_map_change
    def add_wall(self, quarter_tile_pos: Coordinate, side: Side) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        self._mark_quarter_tile(quarter_tile_pos)

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
