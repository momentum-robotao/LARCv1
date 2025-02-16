from enum import Enum
from typing import Literal

LogLevel = Literal["info", "error", "debug", "warning", "critical"]


ALL_LOG_LEVELS: set[LogLevel] = {
    "info",
    "error",
    "debug",
    "warning",
    "critical",
}


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
    rotation_step_by_step = "rotation step by step"
    rotation_angle_correction = "rotation angle correction"
    rotation_angle_correction_kp = "rotation angle correction KP"
    motor_movement = "motor movement"
    movement_step_by_step = "movement step by step"
    motor_velocity = "motor velocity"
    color_sensor_measures = "color sensor measures"
    color_sensor_detections = "color sensor detections"
    maze_snapshot = "maze_snapshot"
    maze_changes = "maze changes"
    maze_answer_str = "maze answer str"
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
    wall_token_classification_image_metrics = "wall token classification: image metrics"
    wall_token_recognition = "wall token recognition"
    hole_detection = "hole detection"
    check_tile_color = "check tile color"
    movement_reason = "movement reason"
    obstacle_detection = "obstacle detection"
    obstacle_avoidance = "obstacle avoidance"


ALL_SYSTEMS = {system for system in System}
