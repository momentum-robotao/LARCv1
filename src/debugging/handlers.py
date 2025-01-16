import logging
import os
import time
from logging import LogRecord

import requests  # type: ignore

from types_and_constants import DEBUG

from .types_and_constants import LogLevel, System


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
                "new_entries": "\n".join(
                    self.format(record) for record in self.new_logs_queue
                )
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
                record.levelname.lower() not in self.levels_to_log
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
                record.levelname.lower() not in self.levels_to_log
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
