import inspect
from functools import wraps

from .create_logger import logger
from .types_and_constants import System


def log_process(
    arguments_to_log: list[str], system_being_debugged: System, from_self: bool = False
):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            sig = inspect.signature(func)
            bound_arguments = sig.bind(*args, **kwargs)
            bound_arguments.apply_defaults()
            arguments_name_to_value_mapper = dict(bound_arguments.arguments)

            arguments_mapper_to_show = {
                name: value
                for name, value in arguments_name_to_value_mapper.items()
                if name in arguments_to_log
            }

            if from_self:
                self_instance = arguments_name_to_value_mapper["self"]
                for argument_name in arguments_to_log:
                    arguments_mapper_to_show[argument_name] = getattr(
                        self_instance, argument_name
                    )

            logger.begin(
                f"Begin - {func.__name__}: {arguments_mapper_to_show}",
                system_being_debugged,
            )

            result = func(*args, **kwargs)

            logger.end(
                f"End - {func.__name__}: {arguments_mapper_to_show}",
                system_being_debugged,
            )

            return result

        return wrapper

    return decorator
