import logging
import os
import time
from enum import Enum
from logging import Logger, LogRecord
from typing import Literal

from types_and_constants import DEBUG, ON_DOCKER


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
    motor_rotation = "motor rotation"
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
    debug_info = "debug info"
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


class DebugInfo:
    def __init__(
        self,
        logger: Logger,
        systems_to_debug: list[System] | None = None,
        systems_to_ignore: list[System] | None = None,
    ) -> None:
        self.systems_to_debug = systems_to_debug or []
        self.systems_to_ignore = systems_to_ignore or []
        self.logger = logger
        if not DEBUG:
            self.systems_to_debug = []
        self.send(
            "Inicializado `DebugInfo`.\n"
            f"Debugar: {self.systems_to_debug}\nIgnorar: {self.systems_to_ignore}",
            System.debug_info,
        )

    def send(
        self,
        message: str,
        system_being_debugged: System,
        level: Literal["info", "error", "debug", "warning", "critical"] = "info",
    ) -> None:
        if system_being_debugged not in self.systems_to_debug:
            if DEBUG and system_being_debugged not in self.systems_to_ignore:
                self.logger.debug(f"{system_being_debugged} => {message}")
            return
        if level in ["error", "critical"]:
            getattr(self.logger, level)(
                f"{system_being_debugged} => {message}", exc_info=True
            )
        else:
            getattr(self.logger, level)(f"{system_being_debugged} => {message}")


if ON_DOCKER:
    import requests  # type: ignore


class HttpHandler(logging.Handler):
    def __init__(
        self,
        url: str,
        entries_between_sends: int = int(os.getenv("ENTRIES_BETWEEN_SENDS", "30")),
    ):
        self.url = url
        logging.Handler.__init__(self=self)
        self.log_queue = ""
        self.log_counter = 0
        self.entries_between_sends = entries_between_sends

    def send_queue_data(self):
        response = requests.post(self.url, json={"new_entries": self.log_queue})
        if response.text != "ok":
            print(
                f"Erro na requisição ao logger: {response.text}. "
                f"Status code: {response.status_code}"
            )
            time.sleep(60)
            self.send_queue_data()
        self.log_queue = ""

    def emit(self, record: LogRecord) -> None:
        self.log_queue += f"{self.format(record)}\n"
        self.log_counter += 1
        if self.log_counter % self.entries_between_sends == 0:
            self.send_queue_data()
