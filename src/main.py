# import sys

# print(sys.path)  # ? para ver path do controller

try:
    import os

    from debugging import DebugInfo
    from end_of_play import end_of_play_routine
    from maze import Maze
    from setup import setup_debuggers, setup_delta_coordinate_mapper, setup_robot
    from solve_map import solve_map
    from types_and_constants import DEBUG, EndOfTimeError, LackOfProgressError

    if DEBUG:
        http_handler, debug_info = setup_debuggers()
    else:
        debug_info = DebugInfo(logger=None)

    robot = setup_robot(debug_info)
    maze = Maze(debug_info)

    try:
        setup_delta_coordinate_mapper(robot, maze)
        solve_map(robot, debug_info, maze)
    except EndOfTimeError:
        pass
    except LackOfProgressError:
        pass
    except Exception:
        if DEBUG:
            from debugging import System

            debug_info.send(
                "Erro inesperado enquanto resolvia o mapa",
                System.unknown_error,
                "critical",
            )
            raise

    end_of_play_routine(robot, maze, debug_info)

    if DEBUG:
        http_handler.send_queue_data()
except Exception as err:
    # ? For debugging purpouses
    DEBUG = (os.getenv("DEBUG", "") + " ").upper()[0] in ["T", "1"]

    if not DEBUG:
        raise

    from controller import Robot as WebotsRobot  # type: ignore

    webots_robot = WebotsRobot()
    webots_robot.step(int(os.getenv("TIME_STEP", 32)))
    print(err)

    import logging

    import requests  # type: ignore

    class HttpHandler(logging.Handler):  # type: ignore
        def __init__(
            self,
            url: str,
        ):
            self.url = url
            logging.Handler.__init__(self=self)

        def emit(self, record: logging.LogRecord) -> None:
            requests.post(self.url, json={"new_entries": f"{self.format(record)}\n"})

    logging.basicConfig()
    logger = logging.getLogger("Robo LARC v1")

    NGROK_URL = ""
    with open("./ngrok.txt", "r") as file:
        NGROK_URL = file.readlines()[0]
    print(f"Url do ngrok recuperada: {NGROK_URL}")

    try:
        resp = requests.post(f"{NGROK_URL}/start_simulation")
        print(resp.text)
    except Exception as e:
        print(f"Erro: {e}")

    http_handler = HttpHandler(f"{NGROK_URL}/send")
    http_handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(levelname)s %(message)s")
    http_handler.setFormatter(formatter)
    logger.addHandler(http_handler)

    logger.critical("erro geral", exc_info=True)
