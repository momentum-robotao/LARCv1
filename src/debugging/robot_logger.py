import logging
from functools import partial
from logging import Logger
from typing import Protocol

import requests  # type: ignore

from types_and_constants import DEBUG

from .handlers import HttpHandler, PrintHandler
from .types_and_constants import ALL_LOG_LEVELS, LogLevel, System


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
            logger.setLevel(logging.DEBUG)
            formatter = logging.Formatter(
                "%(indentation)s%(levelname)s %(system)s\n%(indentation)s%(message)s"
            )

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
        self.indentation = ""

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

    def begin(
        self,
        message: str,
        system_being_debugged: System,
        exc_info: bool | None = None,
    ):
        self._log("info", message, system_being_debugged, exc_info=exc_info)
        self.indentation += "\t"

    def end(
        self,
        message: str,
        system_being_debugged: System,
        exc_info: bool | None = None,
    ):
        self.indentation = self.indentation.removesuffix("\t")
        self._log("info", message, system_being_debugged, exc_info=exc_info)

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
            message,
            exc_info=exc_info,
            extra={"system": system_being_debugged, "indentation": self.indentation},
        )

    def flush(self) -> None:
        if not DEBUG:
            return

        for handler in self.logger.handlers:
            handler.flush()
