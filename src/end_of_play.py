from debugging import System, logger
from maze import Maze
from robot import Robot
from types_and_constants import DEBUG
from utils import delay


def end_of_play_routine(robot: Robot, maze: Maze):
    """Inform the supervisor about the end of a play.
    In the end, we get map bonus.
    """
    try:
        answer_maze = maze.get_answer_maze()
        robot.communicator.send_maze(answer_maze)
        robot.communicator.send_end_of_play()
        delay(robot.webots_robot, 5000)
    except Exception:
        if DEBUG:
            logger.critical(
                "Erro inesperado enquanto finalizava o jogo e enviava o mapa",
                System.unknown_error,
            )
            raise
