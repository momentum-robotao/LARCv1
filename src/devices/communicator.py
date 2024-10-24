import os
import struct
from functools import partial
from typing import NamedTuple

from controller import Robot as WebotsRobot  # type: ignore

from debugging import DebugInfo, System
from helpers import delay
from maze import AnswerMaze
from types_and_constants import (
    DEBUG,
    METER_TO_CM,
    Coordinate,
    LackOfProgressError,
    WallToken,
)

from .device import Device

LACK_OF_PROGRESS_CODE = "L"
END_OF_PLAY_CODE = "E"
GAME_INFORMATION_CODE = "G"
MAP_EVALUATE_REQUEST_CODE = "M"


class GameInformation(NamedTuple):
    score: float
    remaining_simulation_time: int  # in seconds
    remaining_real_world_time: int  # in seconds


class Communicator(Device):
    """
    Robot device used to communicate with Main Supervisor (Erebus' game manager).
    """

    def __init__(
        self,
        robot: WebotsRobot,
        debug_info: DebugInfo,
        emitter_name: str = "emitter",
        receiver_name: str = "receiver",
        time_step: int = int(os.getenv("TIME_STEP", 32)),
        lack_of_progress_code: str = LACK_OF_PROGRESS_CODE,
        end_of_play_code: str = END_OF_PLAY_CODE,
        game_information_code: str = GAME_INFORMATION_CODE,
        map_evaluate_request_code: str = MAP_EVALUATE_REQUEST_CODE,
    ) -> None:
        self._emitter = robot.getDevice(emitter_name)
        self._receiver = robot.getDevice(receiver_name)
        self._receiver.enable(time_step)
        self.debug_info = debug_info
        self.robot_delay = partial(delay, robot, debug_info)

        self.LACK_OF_PROGRESS_CODE = lack_of_progress_code
        self.END_OF_PLAY_CODE = end_of_play_code
        self.GAME_INFORMATION = game_information_code
        self.MAP_EVALUATE_REQUEST_CODE = map_evaluate_request_code

    def send_message(self, message: bytes) -> None:
        if DEBUG:
            self.debug_info.send(
                f"Enviando mensagem {message!r}", System.communicator_send_messages
            )
        self._emitter.send(message)
        if DEBUG:
            self.debug_info.send(
                "Mensagem enviada com sucesso", System.communicator_send_messages
            )

    def get_received_data(self) -> bytes | None:
        if DEBUG:
            self.debug_info.send(
                f"Buscando dados recebidos. Há {self._receiver.getQueueLength()} elementos na fila",
                System.communicator_receive_data,
            )
        if self._receiver.getQueueLength() == 0:
            return None

        received_data = self._receiver.getBytes()
        self._receiver.nextPacket()  # ? Discard the current data packet
        self.debug_info.send(
            f"Dados recebidos: {received_data}", System.communicator_receive_data
        )
        return received_data

    def send_wall_token_information(
        self, position: Coordinate, wall_token: WallToken
    ) -> None:
        """
        Used to report the position and the type of the victim letter or hazard map.
        If correct, points are earned for identifying the game element.
        """
        if DEBUG:
            self.debug_info.send(
                f"Recebido {wall_token} em {position}",
                System.communicator_send_wall_token,
            )
        wall_token_bytes = bytes(wall_token.value, "utf-8")
        x_cm = int(position.x * METER_TO_CM)
        y_cm = int(position.y * METER_TO_CM)

        if DEBUG:
            self.debug_info.send(
                f"Enviando: {wall_token_bytes.decode()} em {x_cm=} e {y_cm=}",
                System.communicator_send_wall_token,
            )
        message = struct.pack("i i c", x_cm, y_cm, wall_token_bytes)
        self.send_message(message)

    def send_lack_of_progress(self) -> None:
        """
        Call lack of progress autonomously.
        """
        if DEBUG:
            self.debug_info.send(
                "Enviando LoP", System.communicator_send_lack_of_progress
            )
        message = struct.pack("c", self.LACK_OF_PROGRESS_CODE.encode())
        self.send_message(message)
        self.robot_delay(
            200
        )  # ? Wait time for robot to respawn and LoP go to receiver queue

    def send_end_of_play(self) -> None:
        """
        Signal to the Main Supervisor the end of play in order to receive an exit bonus.
        """
        if DEBUG:
            self.debug_info.send(
                "Enviando end of play", System.communicator_send_end_of_play
            )
        message = bytes(self.END_OF_PLAY_CODE, "utf-8")
        self.send_message(message)

    def send_maze(self, maze: AnswerMaze) -> None:
        if DEBUG:
            self.debug_info.send(
                "Convertendo mapa para enviá-lo", System.communicator_send_maze
            )
        maze_shape = (len(maze), 0 if len(maze) == 0 else len(maze[0]))
        flat_maze = ",".join([elm for line in maze for elm in line])

        parsed_shape = struct.pack("2i", *maze_shape)
        parsed_maze = flat_maze.encode("utf-8")

        all_bytes = parsed_shape + parsed_maze
        self.send_message(all_bytes)

        if DEBUG:
            self.debug_info.send(
                "Enviando pedido de análise do mapa enviado",
                System.communicator_send_maze,
            )
        map_evaluate_request = struct.pack("c", self.MAP_EVALUATE_REQUEST_CODE.encode())
        self.send_message(map_evaluate_request)

    def occured_lack_of_progress(self) -> bool:
        received_data = self.get_received_data()
        if received_data and len(received_data) == 1:
            first_data_received, *_args = struct.unpack("c", received_data)
            event_code = first_data_received.decode("utf-8")
            if event_code == self.LACK_OF_PROGRESS_CODE:
                return True
        return False

    def get_game_information(self) -> GameInformation:
        if DEBUG:
            self.debug_info.send(
                "Pegando informações da rodada",
                System.communicator_get_game_information,
            )
            self.debug_info.send(
                "Checando por lack of progress na fila do receiver",
                System.communicator_get_game_information,
            )
        if self.occured_lack_of_progress():
            # ? Before emitting data requesting game information, checks for a LoP
            # in order to avoid erasing one from the receiver queue
            raise LackOfProgressError()

        if DEBUG:
            self.debug_info.send(
                "Enviando pedido das informações da rodada",
                System.communicator_get_game_information,
            )

        message = struct.pack("c", self.GAME_INFORMATION.encode())
        self.send_message(message)

        counter = 0
        game_information = None
        while not game_information:
            received_data = self.get_received_data()

            if not received_data or len(received_data) != 16:
                if DEBUG:
                    self.debug_info.send(
                        f"Dados inválidos ou não recebidos: {received_data!r}",
                        System.communicator_get_game_information,
                    )
                self.robot_delay(time_ms=100)
                counter += 1

                if counter == 10:
                    if DEBUG:
                        self.debug_info.send(
                            "Reenviando pedido das informações",
                            System.communicator_get_game_information,
                        )
                    message = struct.pack("c", self.GAME_INFORMATION.encode())
                    self.send_message(message)
                    counter = 0
                continue

            first_data_received, *other_data_received = struct.unpack(
                "c f i i", received_data
            )
            event_code = first_data_received.decode("utf-8")

            if event_code == self.GAME_INFORMATION:
                game_information = GameInformation(*other_data_received)

        if DEBUG:
            self.debug_info.send(
                f"Dados recuperados sobre a partida: {game_information}",
                System.communicator_get_game_information,
            )

        return game_information
