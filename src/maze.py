from functools import wraps
from typing import Any, Callable

from debugging import System, logger
from types_and_constants import (
    ALL_QUADRANTS,
    DEBUG,
    QUADRANT_OF_DELTA,
    Coordinate,
    MappingEncode,
    Quadrant,
    QuarterTile,
    Side,
    SpecialTileType,
    Tile,
)
from utils import quarter_tile_quadrant, tile_pos_with_quarter_tile

MARK_TILES_OF_WALL_SIDE: dict[Side, list[Coordinate]] = {
    "front": [Coordinate(0, 0), Coordinate(1, 0), Coordinate(2, 0)],
    "left": [Coordinate(0, 0), Coordinate(0, 1), Coordinate(0, 2)],
    "right": [Coordinate(2, 0), Coordinate(2, 1), Coordinate(2, 2)],
    "back": [Coordinate(0, 2), Coordinate(1, 2), Coordinate(2, 2)],
}
QUADRANT_DELTA: dict[Quadrant, Coordinate] = (
    {  # delta to a quadrant in matrix-based coordinates
        "top_left": Coordinate(0, 0),
        "top_right": Coordinate(1, 0),
        "bottom_left": Coordinate(0, 1),
        "bottom_right": Coordinate(1, 1),
    }
)

ObjectsMaze = dict[int, dict[int, Tile]]
AnswerMaze = list[list[str]]


def get_needed_map_size(objects_maze: ObjectsMaze) -> tuple[int, int]:
    x_tiles = max(objects_maze) - min(objects_maze) + 1
    y_tiles = (
        max(max(objects_maze[x]) for x in objects_maze)
        - min(min(objects_maze[x]) for x in objects_maze)
        + 1
    )
    return (y_tiles * 4 + 1, x_tiles * 4 + 1)


def log_map_change(func: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(func)
    def wrapper(self: "Maze", *args, **kwargs):
        map_before = str(self)
        result = func(self, *args, **kwargs)
        map_after = str(self)

        if map_before != map_after:
            logger.info(
                f"Mapa modificado para: {map_after}",
                System.maze_snapshot,
            )
        return result

    return wrapper


def reindex_maze(objects: ObjectsMaze) -> ObjectsMaze:
    """Reindex maze in order to match a matrix coordinate instead of plane coordinates.
    All objects in maze has their coordinates reindexed in order to make all coordinates positive.
    The smallest coordinate in each axis is considered the new 0/origin.
    All coordinates are reflected by y because a plane has increasing y to up direction and matrix
    have decreasing y to up direction.
    """
    logger.info(f"Objetos do mapa antes de reindexar: {objects}", System.maze_answer)

    delta_x = 0
    for x in objects:
        delta_x = max(delta_x, 0 - x)
    delta_y = 0
    for x in objects:
        for y in objects[x]:
            delta_y = max(delta_y, 0 - y)

    reindexed_objects: ObjectsMaze = dict()
    for x in objects:
        for y in objects[x]:
            reindexed_objects.setdefault(x + delta_x, dict())[y + delta_y] = objects[x][
                y
            ]
    logger.info(
        f"Objetos do mapa após reindexar: {reindexed_objects}", System.maze_answer
    )

    return reindexed_objects


def insert_walls_to_maze(
    answer_maze: AnswerMaze, tile_pos: Coordinate, tile: Tile
) -> AnswerMaze:
    for quadrant in tile.quadrants:
        quarter_tile_pos = tile_pos + QUADRANT_DELTA[quadrant] * 2
        for wall_in_side in tile.quadrants[quadrant].walls:
            for mark_in_tile in MARK_TILES_OF_WALL_SIDE[wall_in_side]:
                answer_pos = quarter_tile_pos + mark_in_tile
                answer_maze[answer_pos.y][  # type: ignore[index]
                    answer_pos.x  # type: ignore[index]
                ] = MappingEncode.WALL.value
    return answer_maze


def insert_special_tile_to_maze(
    answer_maze: AnswerMaze, tile_pos: Coordinate, tile: Tile
) -> AnswerMaze:
    if tile.special_type in [
        SpecialTileType.STARTING,
        SpecialTileType.HOLE,
        SpecialTileType.PASSAGE_1_2,
        SpecialTileType.PASSAGE_1_3,
        SpecialTileType.PASSAGE_1_4,
        SpecialTileType.PASSAGE_2_3,
        SpecialTileType.PASSAGE_2_4,
        SpecialTileType.PASSAGE_3_4,
        SpecialTileType.SWAMP,
        SpecialTileType.CHECKPOINT,
    ]:
        for mark_in_tile in [
            Coordinate(1, 1),
            Coordinate(1, 3),
            Coordinate(3, 1),
            Coordinate(3, 3),
        ]:
            answer_pos = tile_pos + mark_in_tile
            answer_maze[answer_pos.y][  # type: ignore[index]
                answer_pos.x  # type: ignore[index]
            ] = tile.special_type.value
    return answer_maze


def insert_room4_to_maze(
    answer_maze: AnswerMaze, tile_pos: Coordinate, tile: Tile
) -> AnswerMaze:
    if tile.special_type == SpecialTileType.AREA_4:
        mark = []
        for i in range(5):
            for j in range(5):
                mark.append(Coordinate(i, j))
        for mark_in_tile in mark:
            answer_pos = tile_pos + mark_in_tile
            answer_maze[answer_pos.y][  # type: ignore[index]
                answer_pos.x  # type: ignore[index]
            ] = tile.special_type.value
    return answer_maze


ElementInserterToMaze = Callable[[AnswerMaze, Coordinate, Tile], AnswerMaze]
ELEMENT_INSERTERS: list[ElementInserterToMaze] = [
    insert_special_tile_to_maze,
    insert_walls_to_maze,
    insert_room4_to_maze,
]


class Maze:
    """
    OBS: note that `ObjectsMaze` are composed by full tile coordinates
    while methods from `Maze` deals with quarter-tile coordinates. This
    logic is internal to the class.
    """

    def __init__(self) -> None:
        self.objects: ObjectsMaze = dict()
        self.wall_tokens: list[tuple[Coordinate, Side]] = []

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

    def mark_visited_tile(self, tile_pos: Coordinate) -> None:
        tile_pos = Maze.check_position(tile_pos)
        for quadrant in ALL_QUADRANTS:
            self.objects[tile_pos.x][tile_pos.y].quadrants.setdefault(  # type: ignore[index]
                quadrant, QuarterTile()
            )

    def mark_visited(self, robot_position: Coordinate) -> None:
        """
        :param robot_position: Quarter tile corresponding to robot position.
        """
        for delta, _ in QUADRANT_OF_DELTA.items():
            self._mark_quarter_tile(robot_position + delta)

    def is_visited(self, quarter_tile_pos: Coordinate) -> bool:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)

        tile_pos = tile_pos_with_quarter_tile(quarter_tile_pos)
        quadrant = quarter_tile_quadrant(quarter_tile_pos)
        logger.info(
            f"Checking if tile {tile_pos} is visited in {quadrant=}.",
            System.maze_visited,
        )
        logger.info(f"Already visited in x={tile_pos.x}:", System.maze_visited)
        for y, visited_tile in self.objects.get(tile_pos.x, {}).items():  # type: ignore
            logger.info(f" - {y=}: {visited_tile}", System.maze_visited)
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

        logger.info(
            f"Adicionada parede: {quarter_tile_pos=} e {side=}", System.maze_changes
        )

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
                logger.error(
                    f"Tipo do tile que contém {quarter_tile_pos} "
                    f"era {old_tile_type} e virou {tile_type}",
                    System.maze_changes,
                )
                raise ValueError(
                    f"Tipo do tile que contém {quarter_tile_pos} foi mudado."
                )

        self.objects[tile_pos.x][tile_pos.y].special_type = tile_type  # type: ignore[index]

    def add_wall_token(self, quarter_tile_pos: Coordinate, side: Side) -> None:
        quarter_tile_pos = Maze.check_position(quarter_tile_pos)
        self.wall_tokens.append((quarter_tile_pos, side))

    def get_answer_maze(self) -> AnswerMaze:
        """
        Returns a matrix containing the maze in the format required by Erebus.
        https://v24.erebus.rcj.cloud/docs/rules/mapping/
        - Cada tile 5x5
        * Índice do começo dos tiles é de 5 e 5 por ter borda comum entre eles
        """
        objects = reindex_maze(self.objects)
        map_size = get_needed_map_size(self.objects)
        answer_maze: AnswerMaze = [
            [MappingEncode.DEFAULT.value for i in range(map_size[1])]
            for j in range(map_size[0])
        ]
        logger.info(
            f"Mapa default: {answer_maze}",
            System.maze_answer,
        )

        all_quarter_tiles = set()
        for x in objects:
            for y in objects[x]:
                tile = objects[x][y]
                tile_pos = Coordinate(x, y)
                for quadrant in tile.quadrants:
                    quarter_tile_pos = tile_pos * 2 + QUADRANT_DELTA[quadrant]
                    all_quarter_tiles.add(quarter_tile_pos)

        for x in objects:
            for y in objects[x]:
                tile_pos = Coordinate(x, y) * 4
                tile = objects[x][y]
                for quadrant in tile.quadrants:
                    quarter_tile_pos = tile_pos + QUADRANT_DELTA[quadrant] * 2

                    for delta, wall in [
                        (Coordinate(0, -1), "front"),
                        (Coordinate(0, 1), "back"),
                        (Coordinate(-1, 0), "left"),
                        (Coordinate(1, 0), "right"),
                    ]:
                        if (
                            Coordinate(x, y) * 2 + QUADRANT_DELTA[quadrant] + delta
                            in all_quarter_tiles
                        ):
                            continue
                        for mark_in_tile in MARK_TILES_OF_WALL_SIDE[wall]:  # type: ignore[index]
                            answer_pos = quarter_tile_pos + mark_in_tile
                            answer_maze[answer_pos.y][  # type: ignore[index]
                                answer_pos.x  # type: ignore[index]
                            ] = MappingEncode.WALL.value
        logger.info(f"Mapa após paredes externas: {answer_maze}", System.maze_answer)

        for inserter in ELEMENT_INSERTERS:
            for x in objects:
                for y in objects[x]:
                    tile_pos = Coordinate(x, y) * 4
                    tile = objects[x][y]
                    answer_maze = inserter(answer_maze, tile_pos, tile)
            logger.info(
                f"Mapa após {inserter.__name__}: {answer_maze}", System.maze_answer
            )

        return answer_maze
