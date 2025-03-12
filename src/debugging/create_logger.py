from .robot_logger import RobotLogger
from .types_and_constants import ALL_LOG_LEVELS, System

systems = {
    #     System.check_tile_color,
    #     System.color_sensor_detections,
    #     System.color_sensor_measures,
    # System.communicator_get_game_information,
    # System.communicator_receive_data,
    # System.communicator_send_end_of_play,
    # System.communicator_send_lack_of_progress,
    # System.communicator_send_maze,
    # System.communicator_send_messages,
    # System.communicator_send_wall_token,
    # System.delay,
    # System.lidar_side_measures,
    System.dfs_decision,
    System.dfs_state,
    System.dfs_verification,
    # System.gps_measures,
    # System.hole_detection,
    System.imu_measures,
    # System.initialization,
    System.maze_answer_str,
    # System.wall_token_classification,
    # System.wall_token_classification_image_metrics,
    # System.wall_token_recognition,
    # System.obstacle_avoidance,
    # System.obstacle_detection,
    System.unknown_error,
    System.motor_movement,
    System.movement_step_by_step,
    System.movement_reason,
    # System.rotation,
    # System.rotation_step_by_step,
    # System.rotation_angle_correction,
}

logger = RobotLogger(
    systems_to_log_in_file=systems,
    levels_to_log_in_file=ALL_LOG_LEVELS,
    systems_to_print_logs=systems
    - {System.lidar_side_measures, System.lidar_side_measures},
    levels_to_print_logs=ALL_LOG_LEVELS,
)
