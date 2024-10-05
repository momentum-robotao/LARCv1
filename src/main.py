import logging
import os
from datetime import datetime

from controller import Robot as WebotsRobot  # type: ignore

from debugging import ALL_SYSTEMS, DebugInfo, HttpHandler, System
from devices import GPS, IMU, ColorSensor, Lidar, Motor
from dfs import dfs
from maze import Maze
from robot import Robot
from types_and_constants import DEBUG, NGROK_URL, ON_DOCKER, Coordinate

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


def solve_map(robot: Robot, debug_info: DebugInfo) -> None:
    maze = Maze(debug_info)

    position = Coordinate(0, 0)
    position = dfs(position, maze, robot, debug_info, area=1)
    # TODO: transition between maps
    # position = dfs(position, maze, all_robot_info, area=2)


def main() -> None:
    # Initialize DebugInfo instance
    if DEBUG:
        logger.info(f"Começando nova execução: {datetime.now()}")

    try:
        # debug_info = DebugInfo(
        #     systems_to_debug=[
        #         e for e in ALL_SYSTEMS if str(e) not in [str(System.lidar_measures)]
        #     ],
        #     systems_to_ignore=[System.lidar_measures],
        # )
        want = [
            System.dfs_state,
            System.dfs_decision,
            System.dfs_verification,
            System.maze_visited,
            System.unknown_error,
        ]
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

        robot = Robot(webots_robot, motor, lidar, gps, imu, color_sensor, debug_info)
    except Exception:
        if DEBUG:
            debug_info.send(
                "Erro durante inicialização do robô", System.initialization, "error"
            )
            raise

    # Solve map
    try:
        # while robot.step() != -1:
        #     print(robot.imu.get_rotation_angle())
        solve_map(robot, debug_info)
    except Exception:
        if DEBUG:
            debug_info.send(
                "Erro inesperado enquanto resolvia o mapa",
                System.unknown_error,
                "critical",
            )
            raise


try:
    main()
except Exception:
    pass
if ON_DOCKER:
    http_handler.send_queue_data()
