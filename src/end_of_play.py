from debugging import DebugInfo, System
from helpers import delay
from maze import Maze
from robot import Robot
from types_and_constants import DEBUG


def end_of_play_routine(robot: Robot, maze: Maze, debug_info: DebugInfo):
    """Inform the supervisor about the end of a play.
    In the end, we get map bonus.
    """
    try:
        answer_maze = maze.get_answer_maze()
        robot.communicator.send_maze(answer_maze)
        robot.communicator.send_end_of_play()
        delay(robot.webots_robot, debug_info, 5000)
    except Exception:
        if DEBUG:
            debug_info.send(
                "Erro inesperado enquanto finalizava o jogo e enviava o mapa",
                System.unknown_error,
                "critical",
            )
            raise
