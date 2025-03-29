from debugging import DebugInfo
from dfs import dfs
from maze import Maze
from robot import Robot
from types_and_constants import SLOW_DOWN_DIST, Coordinate, SpecialTileType
from slam_navigation import navigation
import numpy as np


def solve_map(robot: Robot, debug_info: DebugInfo, maze: Maze) -> None:
    initial_position = Coordinate(0, 0)
    maze.set_tile_type(initial_position, SpecialTileType.STARTING)
    transitions = dfs(
        initial_position,
        maze,
        robot,
        debug_info,
        area=1,
        moves_before=[],
        starting=True,
    )
    best_transition_pos, best_transition, best_moves_before = None, None, None
    priority = [
        SpecialTileType.PASSAGE_2_4,
        SpecialTileType.PASSAGE_3_4,
        SpecialTileType.PASSAGE_1_4,
        None,
    ]
    # print(transitions)
    for transition_pos, transition, moves_before in transitions:
        if priority.index(transition) < priority.index(best_transition):
            best_transition_pos, best_transition, best_moves_before = (
                transition_pos,
                transition,
                moves_before,
            )
    # print(f"Melhor: {best_transition}")
    if best_transition is None:
        return

    # go to room 4
    print("Indo para sala 4")
    
    for move in best_moves_before:
        command, args = move
        if command == "rotate_to_angle":
            robot.rotate_to_angle(args[0])
        if command == "move":
            robot.move(
                args[0],    
                Maze(debug_info),
                dist=args[1],
                slow_down_dist=SLOW_DOWN_DIST / 3,
            )
    
    print("Chegou na sala 4")

    navigation.slam_dfs(robot.gps.get_position_AUGUSTO(), robot.lidar.get_distances_by_side_angle_AUGUSTO(), robot.imu.get_rotation_angle(), robot, maze)
    ''' 
    dfs(
        best_transition_pos,
        maze,
        robot,
        debug_info,
        area=4,
        moves_before=[],
        starting=True,
    )
    '''

    


    # TODO4: retornar pra tile inicial (+10%)
    # best_moves_before.reverse()
    # for move in best_moves_before:
    #     command, args = move
    #     if command == "rotate_to_angle":
    #         robot.rotate_to_angle(args[0])
    #     if command == "move":
    #         robot.move(
    #             "backward",
    #             Maze(debug_info),
    #             dist=args[1],
    #             slow_down_dist=SLOW_DOWN_DIST / 3,
    #         )
