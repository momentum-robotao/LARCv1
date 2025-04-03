"""
Regras importantes:
    Próximo
    "For successful wall token identification (TI) (40], the center of the robot must be
    equal to or less than half a tile distance from the location of the wall token
    when the robot indicates a wall token has been identified."

    Parado
    "To identify a wall token, the robot must stop at each one for 1 second.
    After 1 second, it must send a command to the game manager with the type
    of the wall token in a platform-specific format"
"""

from math import atan, cos, sin, sqrt

import cv2 as cv
import numpy as np
from ultralytics import YOLO  # type: ignore[import-untyped]

from devices import GPS, IMU, Camera
from types_and_constants import ROBOT_RADIUS, TILE_SIZE, Coordinate, WallToken

MODEL_PATH = r"/usr/local/controller/best.pt"
CALIBRATION_PATH = r"/usr/local/controller/calibration.npz"
MIN_DIST_TO_SEND_WT = TILE_SIZE / 2 - 0.01
MIN_ACCURACY_TO_CONSIDER_WT = 0.85
CAMERA_FOCUS_RADIUS = (
    ROBOT_RADIUS - 0.005
)  # ? radius - dist(robot side, camera optical center)

model = YOLO(MODEL_PATH)

with np.load(CALIBRATION_PATH) as calibration_file:
    camMatrix, distCoeff, _, _ = [
        calibration_file[i] for i in ("camMatrix", "distCoeff", "rvecs", "tvecs")
    ]


def get_camera_focus_coordinate(
    robot_position: Coordinate, rotation_angle: float
) -> Coordinate:
    """
    :param robot_position: In GPS coordinates.
    :param rotation_angle: In GPS axes.

    Returns camera focus coordinate in GPS axes.
    """
    dx = CAMERA_FOCUS_RADIUS * cos(rotation_angle)
    dy = CAMERA_FOCUS_RADIUS * sin(rotation_angle)
    camera_focus = robot_position + Coordinate(dx, dy)
    return camera_focus


def get_WT_coordinate_deltas(
    tvec_dx: float, tvec_dz: float, rotation_angle: float
) -> tuple[float, float]:
    """
    :param rotation_angle: In GPS axes.

    Returns (dx, dy) in GPS axes.
    """
    side_angle_WT = atan(tvec_dx / tvec_dz)
    angle_WT = rotation_angle + side_angle_WT
    distance_WT = sqrt(tvec_dx * tvec_dx + tvec_dz * tvec_dz)
    dx = distance_WT * cos(angle_WT)
    dy = distance_WT * sin(angle_WT)

    return (dx, dy)


def classify_wall_token(
    camera: Camera,
    gps: GPS,
    imu: IMU,
) -> WallToken | None:
    wall_token: WallToken | None = None

    image = camera.get_rgb_matrix()
    results = model(image)

    class_names = model.names

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = class_names[class_id]

            print(f"Acurácia {class_name}: {conf * 100}%")

            if conf < MIN_ACCURACY_TO_CONSIDER_WT:
                continue

            corners = ((x1, y1), (x2, y1), (x2, y2), (x1, y2))

            camera_position = get_camera_focus_coordinate(
                robot_position=gps.get_position(), rotation_angle=imu.get_GPS_angle()
            )

            obj_points = np.array(
                (
                    (0.008025, -0.008025, 0),
                    (-0.008025, -0.008025, 0),
                    (-0.008025, 0.008025, 0),
                    (0.008025, 0.008025, 0),
                ),
                dtype=np.float32,
            )
            bounding_box = np.array(corners, dtype=np.float32).reshape((4, 1, 2))
            ret, rvecs, tvecs = cv.solvePnP(
                obj_points, bounding_box, camMatrix, distCoeff
            )
            tvec_dx, _dy, tvec_dz = tvecs.ravel()

            dx, dy = get_WT_coordinate_deltas(
                tvec_dx, tvec_dz, rotation_angle=imu.get_GPS_angle()
            )

            print(
                f"Encontrado {class_name}: {camera_position.x + dx}, {camera_position.y + dy}"
            )

    return wall_token
