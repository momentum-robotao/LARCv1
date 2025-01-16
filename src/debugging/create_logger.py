from .robot_logger import RobotLogger
from .types_and_constants import ALL_LOG_LEVELS, ALL_SYSTEMS

logger = RobotLogger(
    systems_to_log_in_file=ALL_SYSTEMS, levels_to_log_in_file=ALL_LOG_LEVELS
)
