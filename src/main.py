# import sys

# print(sys.path)  # ? para ver path do controller

try:
    import logging
    import os
    from datetime import datetime

    from controller import Robot as WebotsRobot  # type: ignore

    from debugging import ALL_SYSTEMS, DebugInfo, HttpHandler, System
    from devices import (
        GPS,
        IMU,
        Camera,
        ColorSensor,
        Communicator,
        DistanceSensor,
        Lidar,
        Motor,
    )
    from dfs import dfs
    from helpers import delay
    from maze import Maze
    from robot import Robot
    from types_and_constants import (
        DEBUG,
        NGROK_URL,
        ON_DOCKER,
        Coordinate,
        SpecialTileType,
    )

    if ON_DOCKER:
        import requests  # type: ignore

    # Initialize logger
    try:
        log_dir = os.path.dirname(os.getenv("LOG_PATH", ""))
        os.makedirs(log_dir, exist_ok=True)
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(levelname)s %(message)s",
            filename=os.getenv("LOG_PATH"),
        )
        logger = logging.getLogger("Robo LARC v1")
        logger.info(f"Criado log com sucesso em: {os.getenv('LOG_PATH')}")
        if ON_DOCKER:
            print(f"Url do ngrok recuperada: {NGROK_URL}")

            requests.post(f"{NGROK_URL}/start_simulation")

            http_handler = HttpHandler(f"{NGROK_URL}/send")
            http_handler.setLevel(logging.DEBUG)
            formatter = logging.Formatter("%(levelname)s %(message)s")
            http_handler.setFormatter(formatter)
            logger.addHandler(http_handler)

            print("Adicionado handler com ngrok")

    except Exception:
        if DEBUG:
            logging.error("Erro ao inicializar o logger", exc_info=True)
            raise

    def solve_map(robot: Robot, debug_info: DebugInfo, maze: Maze) -> None:
        robot.imu.get_rotation_angle()
        position = Coordinate(0, 0)
        maze.set_tile_type(position, SpecialTileType.STARTING)
        position = dfs(position, maze, robot, debug_info, area=1, starting=True)

    def main() -> None:
        # Initialize DebugInfo instance
        if DEBUG:
            logger.info(f"Começando nova execução: {datetime.now()}")

        try:
            want = [
                System.dfs_state,
                System.dfs_decision,
                System.dfs_verification,
                System.unknown_error,
                System.initialization,
                System.maze_changes,
                System.maze_answer,
                System.maze_visited,
                System.communicator_send_maze,
                System.communicator_send_end_of_play,
                System.communicator_send_messages,
                System.wall_token_recognition,
                System.wall_token_classification,
                System.hole_detection,
            ]
            # want = ALL_SYSTEMS
            debug_info = DebugInfo(
                logger,
                systems_to_debug=want,
                systems_to_ignore=[
                    e for e in ALL_SYSTEMS if str(e) not in [str(w) for w in want]
                ],
            )
        except Exception:
            if DEBUG:
                logger.error("Erro ao inicializar o debug info", exc_info=True)
                raise

        # Initialize robot and devices
        try:
            webots_robot = WebotsRobot()
            webots_robot.step(int(os.getenv("TIME_STEP", 32)))

            motor = Motor(webots_robot, debug_info)
            lidar = Lidar(webots_robot, debug_info)
            gps = GPS(webots_robot, debug_info)
            imu = IMU(webots_robot, debug_info)
            color_sensor = ColorSensor(webots_robot, debug_info)
            communicator = Communicator(webots_robot, debug_info)
            camera = Camera(webots_robot, debug_info)
            distance_sensor = DistanceSensor(webots_robot, debug_info)
        except Exception:
            if DEBUG:
                debug_info.send(
                    "Erro durante inicialização dos devices",
                    System.initialization,
                    "error",
                )
                raise

        try:
            robot = Robot(
                webots_robot,
                motor,
                lidar,
                gps,
                imu,
                color_sensor,
                communicator,
                camera,
                distance_sensor,
                debug_info=debug_info,
            )
            robot.step()
        except Exception:
            if DEBUG:
                debug_info.send(
                    "Erro durante inicialização do robô", System.initialization, "error"
                )
                raise

        maze = Maze(debug_info)

        # Solve map
        try:
            solve_map(robot, debug_info, maze)
        except Exception:
            if DEBUG:
                debug_info.send(
                    "Erro inesperado enquanto resolvia o mapa",
                    System.unknown_error,
                    "critical",
                )
                raise

        # Routine to inform supervisor about the end of play. In the end, we get map bonus
        try:
            answer_maze = maze.get_answer_maze()
            communicator.send_maze(answer_maze)
            communicator.send_end_of_play()
            delay(webots_robot, debug_info, 5000)
        except Exception as err:
            if DEBUG:
                debug_info.send(
                    "Erro inesperado enquanto finalizava o jogo e enviava o mapa",
                    System.unknown_error,
                    "critical",
                )
                print(f"Erro ao finalizar jogo e enviar mapa: {err}")
                raise

    try:
        main()
    except Exception:
        pass
    http_handler.send_queue_data()
except Exception as err:
    from controller import Robot as WebotsRobot  # type: ignore

    webots_robot = WebotsRobot()
    webots_robot.step(int(os.getenv("TIME_STEP", 32)))
    print(err)

    if (os.getenv("ON_DOCKER", "") + " ").upper()[0] in ["T", "1"]:
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
                requests.post(
                    self.url, json={"new_entries": f"{self.format(record)}\n"}
                )

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
