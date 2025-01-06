import os

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
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
from maze import Maze
from robot import Robot, set_dist_change_mapper
from types_and_constants import DEBUG, DEGREE_IN_RAD


def setup_debuggers() -> tuple:
    import logging
    from datetime import datetime

    import requests  # type: ignore

    from debugging import ALL_SYSTEMS, HttpHandler, System

    with open("./ngrok.txt", "r") as file:
        NGROK_URL = file.readlines()[0]

    # Initialize logger
    try:
        print(f"Inicializando logger em: {os.getenv('LOG_PATH')}")
        log_dir = os.path.dirname(os.getenv("LOG_PATH", ""))
        os.makedirs(log_dir, exist_ok=True)
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(levelname)s %(message)s",
            filename=os.getenv("LOG_PATH"),
        )
        logger = logging.getLogger("Robo LARC v1")
        print("logger inicializado")

        print(f"Url do ngrok recuperada: {NGROK_URL}")
        requests.post(f"{NGROK_URL}/start_simulation")

        http_handler = HttpHandler(f"{NGROK_URL}/send")
        http_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter("%(levelname)s %(message)s")
        http_handler.setFormatter(formatter)
        logger.addHandler(http_handler)
        print("conexão com ngrok feita")

    except Exception as err:
        print("Erro ao inicializar o logger", err)
        raise

    # Initialize DebugInfo
    logger.info(f"Começando nova execução: {datetime.now()}")
    try:
        want = [
            System.unknown_error,
            System.initialization,
        ]
        debug_info = DebugInfo(
            logger,
            systems_to_debug=want,
            systems_to_ignore=[
                e for e in ALL_SYSTEMS if str(e) not in [str(w) for w in want]
            ],
        )
    except Exception:
        logger.error("Erro ao inicializar o debug info", exc_info=True)
        raise

    return http_handler, debug_info


def setup_robot(debug_info: DebugInfo) -> Robot:
    # Initialize robot and devices
    try:
        webots_robot = WebotsRobot()
        webots_robot.step(int(os.getenv("TIME_STEP", 32)))
        lidar = Lidar(webots_robot, debug_info)
        gps = GPS(webots_robot, debug_info)
        motor = Motor(webots_robot, debug_info)
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
    return robot


def setup_delta_coordinate_mapper(robot: Robot, maze: Maze):
    robot.imu.get_rotation_angle()
    initial_position = robot.gps.get_position()
    robot.move(
        "forward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    delta_0_cord = robot.gps.get_position() - initial_position
    delta_0 = (delta_0_cord.x, delta_0_cord.y)
    robot.move(
        "backward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    robot.rotate("right", 45 * DEGREE_IN_RAD, correction_rotation=True)
    initial_position = robot.gps.get_position()
    robot.move(
        "forward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    delta_45_cord = robot.gps.get_position() - initial_position
    delta_45 = (delta_45_cord.x, delta_45_cord.y)
    robot.move(
        "backward",
        maze,
        dist=0.01,
        correction_move=True,
    )
    robot.rotate("left", 45 * DEGREE_IN_RAD, correction_rotation=True)
    set_dist_change_mapper(delta_0, delta_45)
