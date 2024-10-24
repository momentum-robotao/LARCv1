from functools import wraps
from typing import Any, Callable

from debugging import DebugInfo, System
from helpers import quarter_tile_quadrant, tile_pos_with_quarter_tile
from types_and_constants import (
    DEBUG,
    DELTA_TO_QUADRANT,
    MAP_SIZE,
    Coordinate,
    MappingEncode,
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


def reindex_maze(
    objects: dict[int, dict[int, Tile]], debug_info: DebugInfo
) -> dict[int, dict[int, Tile]]:
    """Reindex maze in order to remove negative coordinates.
    All objects in maze has their coordinates reindexed in order to make all coordinates positive.
    The smallest coordinate in each axis is considered the new 0/origin.
    """
    if DEBUG:
        debug_info.send(
            f"Objetos do mapa antes de reindexar: {objects}", System.maze_answer
        )
    delta_x = 0
    for x in objects:
        delta_x = max(delta_x, 0 - x)
    delta_y = 0
    for x in objects:
        for y in objects[x]:
            delta_y = max(delta_y, 0 - y)

    reindexed_objects: dict[int, dict[int, Tile]] = dict()
    for x in objects:
        for y in objects[x]:
            reindexed_objects.setdefault(x + delta_x, dict())[y + delta_y] = objects[x][
                y
            ]
    if DEBUG:
        debug_info.send(
            f"Objetos do mapa após reindexar: {reindexed_objects}", System.maze_answer
        )

    return reindexed_objects


class Maze:
    """
    OBS: note that `ObjectsMaze` are composed by full tile coordinates
    while methods from `Maze` deals with quarter-tile coordinates. This
    logic is internal to the class.
    """

    def __init__(self, debug_info: DebugInfo) -> None:
        self.objects: dict[int, dict[int, Tile]] = dict()
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
        self.objects.setdefault(tile_pos.x, {}).setdefault(  # type: ignore[arg-type]
            tile_pos.y, Tile()  # type: ignore[arg-type]
        )
        self.objects[tile_pos.x][tile_pos.y].quadrants.setdefault(  # type: ignore[index]
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
        if DEBUG:
            self.debug_info.send(
                f"Checking if tile {tile_pos} is visited in {quadrant=}.",
                System.maze_visited,
            )
            self.debug_info.send(
                f"Already visited in x={tile_pos.x}:", System.maze_visited
            )
            for y, visited_tile in self.objects.get(tile_pos.x, {}).items():  # type: ignore
                self.debug_info.send(f" - {y=}: {visited_tile}", System.maze_visited)
        return (
            quadrant
            in self.objects.get(tile_pos.x, {}).get(tile_pos.y, Tile()).quadrants  # type: ignore
        )

    @log_map_change
    def add_wall(self, quarter_tile_pos: Coordinate, side: Side) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        self._mark_quarter_tile(quarter_tile_pos)

        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)
        self.objects[tile_pos.x][tile_pos.y].quadrants[  # type: ignore[index]
            quarter_tile_quadrant(quarter_tile_pos)
        ].walls.add(side)

    @log_map_change
    def set_tile_type(
        self,
        quarter_tile_pos: Coordinate,
        tile_type: SpecialTileType,
    ) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        self._mark_quarter_tile(quarter_tile_pos)
        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)

        old_tile_type = self.objects[tile_pos.x][tile_pos.y].special_type  # type: ignore[index]
        if old_tile_type is not None and old_tile_type != tile_type:
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

        self.objects[tile_pos.x][tile_pos.y].special_type = tile_type  # type: ignore[index]

    def add_wall_token(self, quarter_tile_pos: Coordinate, side: Side) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        self.wall_tokens.append((quarter_tile_pos, side))

    # TODO: refatorar talvez com função que adiciona máscaras pra cada tile 5x5
    # # e aplica tipo starting tile...
    def get_answer_maze(self) -> AnswerMaze:
        """
        Returns a matrix containing the maze in the format required by Erebus.
        https://v24.erebus.rcj.cloud/docs/rules/mapping/
        - Cada tile 5x5
        """
        self.objects = reindex_maze(self.objects, self.debug_info)
        answer_maze: AnswerMaze = [
            [MappingEncode.DEFAULT.value for i in range(MAP_SIZE[0])]
            for j in range(MAP_SIZE[1])
        ]
        self.debug_info.send(
            f"Mapa default: {answer_maze}",
            System.maze_answer,
        )

        for x in self.objects:
            for y in self.objects[x]:
                if self.objects[x][y].special_type == SpecialTileType.STARTING:
                    for mark_in_tile in [(1, 1), (1, 3), (3, 1), (3, 3)]:
                        x_answer = 5 * x + mark_in_tile[0]
                        y_answer = 5 * y + mark_in_tile[1]
                        answer_maze[x_answer][y_answer] = SpecialTileType.STARTING.value
        self.debug_info.send(
            f"Mapa após tile inicial: {answer_maze}", System.maze_answer
        )

        return answer_maze
