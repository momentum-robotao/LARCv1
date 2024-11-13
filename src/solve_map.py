from debugging import DebugInfo
from dfs import dfs
from maze import Maze
from robot import Robot
from types_and_constants import Coordinate, SpecialTileType


def solve_map(robot: Robot, debug_info: DebugInfo, maze: Maze) -> None:
    position = Coordinate(0, 0)
    maze.set_tile_type(position, SpecialTileType.STARTING)
    position = dfs(position, maze, robot, debug_info, area=1, starting=True)
