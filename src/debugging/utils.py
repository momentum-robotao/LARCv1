import inspect
from functools import wraps

from .robot_logger import RobotLogger
from .types_and_constants import System


# TODO: turn logger into a singleton
def log_process(arguments_to_log: list[str], system_being_debugged: System):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            sig = inspect.signature(func)
            bound_arguments = sig.bind(*args, **kwargs)
            bound_arguments.apply_defaults()
            arguments_name_to_value_mapper = dict(bound_arguments.arguments)

            logger: RobotLogger | None = arguments_name_to_value_mapper.get(  # type: ignore
                "logger"
            )
            if logger is None and "self" in arguments_name_to_value_mapper:
                logger: RobotLogger = arguments_name_to_value_mapper["self"].logger  # type: ignore

            arguments_mapper_to_show = {
                name: value
                for name, value in arguments_name_to_value_mapper.items()
                if name in arguments_to_log
            }

            if logger is not None:
                logger.begin(
                    f"Begin - {func.__name__}: {arguments_mapper_to_show}",
                    system_being_debugged,
                )

            result = func(*args, **kwargs)

            if logger is not None:
                logger.end(
                    f"End - {func.__name__}: {arguments_mapper_to_show}",
                    system_being_debugged,
                )

            return result

        return wrapper

    return decorator
