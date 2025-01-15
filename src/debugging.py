import logging
import os
import time
from enum import Enum
from functools import partial
from logging import Logger, LogRecord
from typing import Literal, Protocol

from types_and_constants import DEBUG

if DEBUG:
    import requests  # type: ignore


LogLevel = Literal["info", "error", "debug", "warning", "critical"]


ALL_LOG_LEVELS: list[LogLevel] = ["info", "error", "debug", "warning", "critical"]


class System(Enum):
    initialization = "initialization"
    unknown_error = "unknown error"
    delay = "delay"
    lidar_measures = "lidar measures"
    lidar_range_measures = "lidar range measures"
    lidar_side_measures = "lidar side measures"
    lidar_wall_detection = "lidar wall detection"
    gps_measures = "gps measures"
    imu_measures = "imu measures"
    rotation = "motor rotation"
    motor_movement = "motor movement"
    motor_velocity = "motor velocity"
    rotation_angle_correction = "rotation angle correction"
    color_sensor_measures = "color sensor measures"
    color_sensor_detections = "color sensor detections"
    maze_snapshot = "maze_snapshot"
    maze_changes = "maze changes"
    dfs_state = "dfs state"
    dfs_verification = "dfs verification"
    dfs_decision = "dfs decision"
    maze_visited = "maze visited"
    maze_answer = "maze answer"
    communicator_send_messages = "communicator send messages"
    communicator_send_wall_token = "communicator send wall token"
    communicator_send_lack_of_progress = "communicator send lack of progress"
    communicator_send_end_of_play = "communicator send end of play"
    communicator_receive_data = "communicator receive data"
    communicator_get_game_information = "communicator get game information"
    communicator_send_maze = "communicator send maze"
    wall_token_classification = "wall token classification"
    wall_token_recognition = "wall token recognition"
    hole_detection = "hole detection"
    check_tile_color = "check tile color"


ALL_SYSTEMS = [system for system in System]


class LogCallable(Protocol):
    def __call__(
        self, message: str, system_being_debugged: System, exc_info: bool | None = None
    ) -> None:
        """
        Callable for logging a message at a specific level.

        Parameters:
        - message: The log message to be recorded.
        - system_being_debugged: The system associated with the message.
        - exc_info: If traceback from current exceptions should be logged.
        """
        ...


class HttpHandler(logging.Handler):
    def __init__(
        self,
        url: str,
        entries_between_sends: int = int(os.getenv("ENTRIES_BETWEEN_SENDS", "30")),
        systems_to_log: set[System] | None = None,
        levels_to_log: set[LogLevel] | None = None,
    ) -> None:
        if not DEBUG:
            return

        logging.Handler.__init__(self=self)

        self.url = url
        self.new_logs_queue: list[LogRecord] = []
        self.entries_between_sends = entries_between_sends
        self.systems_to_log = systems_to_log or set()
        self.levels_to_log = levels_to_log or set()

    def flush(self):
        if not DEBUG:
            return

        response = requests.post(
            self.url,
            json={
                "new_entries": "\n".join(str(record) for record in self.new_logs_queue)
            },
        )
        if response.text != "ok":
            print(
                f"Erro na requisição ao logger: {response.text}. "
                f"Status code: {response.status_code}"
            )
            time.sleep(20)
            self.flush()
        self.new_logs_queue.clear()

    def emit(self, record: LogRecord) -> None:
        if not DEBUG:
            return

        try:
            if (
                record.levelname not in self.levels_to_log
                or record.system not in self.systems_to_log  # type: ignore[attr-defined]
            ):
                return

            self.new_logs_queue.append(record)
            if len(self.new_logs_queue) >= self.entries_between_sends:
                self.flush()
        except Exception:
            self.handleError(record)


class PrintHandler(logging.Handler):
    def __init__(
        self,
        systems_to_log: set[System] | None = None,
        levels_to_log: set[LogLevel] | None = None,
    ):
        if not DEBUG:
            return

        logging.Handler.__init__(self=self)

        self.systems_to_log = systems_to_log or set()
        self.levels_to_log = levels_to_log or set()

    def emit(self, record):
        """
        Emit a record using the print function.
        """
        try:
            if (
                record.levelname not in self.levels_to_log
                or record.system not in self.systems_to_log
            ):
                return

            msg = self.format(record)
            print(msg)
            self.flush()
        except Exception:
            self.handleError(record)

    def flush(self):
        print("", end="", flush=True)


class RobotLogger:
    def __init__(
        self,
        logger: Logger | None = None,
        logger_name: str = "Robo LARC v1",
        systems_to_log_in_file: set[System] | None = None,
        levels_to_log_in_file: set[LogLevel] | None = None,
        systems_to_print_logs: set[System] | None = None,
        levels_to_print_logs: set[LogLevel] | None = None,
    ) -> None:
        if not DEBUG:
            return

        if logger is None:
            logger = logging.getLogger(logger_name)
            formatter = logging.Formatter("%(levelname)s %(system)s => %(message)s")

            print_handler = PrintHandler(
                systems_to_log=systems_to_print_logs, levels_to_log=levels_to_print_logs
            )
            print_handler.setLevel(logging.DEBUG)
            print_handler.setFormatter(formatter)

            logger.addHandler(print_handler)

            with open("./ngrok.txt", "r") as file:
                NGROK_URL = file.readline()

            http_handler = HttpHandler(
                f"{NGROK_URL}/send",
                systems_to_log=systems_to_log_in_file,
                levels_to_log=levels_to_log_in_file,
            )
            http_handler.setLevel(logging.DEBUG)
            http_handler.setFormatter(formatter)

            requests.post(f"{NGROK_URL}/start_simulation")
            print(f"Connected to Ngrok: {NGROK_URL}")

            logger.addHandler(http_handler)
        self.logger = logger

        self.info(
            "Initialized `RobotLogger`.",
            System.initialization,
        )
        if logger is None:
            self.info(
                f" - {systems_to_log_in_file=}\n - {levels_to_log_in_file=}\n"
                f" - {systems_to_print_logs=}\n - {levels_to_print_logs=}",
                System.initialization,
            )

    def __getattr__(self, attribute_name: str) -> LogCallable:
        if attribute_name not in ALL_LOG_LEVELS:
            raise AttributeError()

        level: LogLevel = attribute_name  # type: ignore

        return partial(
            self._log,
            level,
        )

    def _log(
        self,
        level: LogLevel,
        message: str,
        system_being_debugged: System,
        exc_info: bool | None = None,
    ) -> None:
        if not DEBUG:
            return

        if exc_info is None:
            exc_info = level in ["error", "critical"]

        getattr(self.logger, level)(
            message, exc_info=exc_info, extra={"system": system_being_debugged}
        )

    def flush(self) -> None:
        if not DEBUG:
            return

        for handler in self.logger.handlers:
            handler.flush()
