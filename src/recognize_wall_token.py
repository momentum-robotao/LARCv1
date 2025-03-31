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
from numpy.typing import NDArray
from ultralytics import YOLO  # type: ignore[import-untyped]

from devices import GPS, IMU, Camera
from types_and_constants import (
    DEGREE_IN_RAD,
    ROBOT_RADIUS,
    TILE_SIZE,
    Coordinate,
    WallToken,
)
from utils import cyclic_angle

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
    print(f"Foco da câmera em: {camera_focus}")
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
    print(f"{side_angle_WT / DEGREE_IN_RAD}")
    print(f"tvec: {tvec_dx}, {tvec_dz}")
    print(f"deltas: {dx}, {dy}")

    return (dx, dy)


def get_hsv_background(
    rgb_image: NDArray[np.uint8], x1: int, y1: int, x2: int, y2: int
) -> NDArray[np.uint8] | None:
    """
    Tries to get hsv color of background surrounding the object in bounding box.

    :param x1,y1,x2,y2: bounding box
    """
    image = cv.cvtColor(rgb_image, cv.COLOR_RGB2HSV)
    height, width, _ = image.shape

    if x2 + 5 < width:
        color1 = image[y1, x2 + 5]
        color2 = image[y2, x2 + 5]
        if all(color1 == color2):
            return color1.ravel()

    if y2 + 5 < height:
        color1 = image[y2 + 5, x1]
        color2 = image[y2 + 5, x2]
        if all(color1 == color2):
            return color1.ravel()

    if y1 >= 5:
        color1 = image[y1 - 5, x1]
        color2 = image[y1 - 5, x2]
        if all(color1 == color2):
            return color1.ravel()

    if x1 >= 5:
        color1 = image[y1, x1 - 5]
        color2 = image[y2, x1 - 5]
        if all(color1 == color2):
            return color1.ravel()
    cv.imwrite("not_found.png", rgb_image)
    cv.imwrite(
        "not_found_cropped.png",
        rgb_image[
            max(0, y1 - 5) : min(height, y2 + 6), max(0, x1 - 5) : min(width, x2 + 6)
        ],
    )
    cv.imwrite(
        "not_found_cropped_hsv.png",
        image[
            max(0, y1 - 5) : min(height, y2 + 6), max(0, x1 - 5) : min(width, x2 + 6)
        ],
    )

    return None


def get_corners(
    rgb_image: NDArray[np.uint8],
    x1: int,
    y1: int,
    x2: int,
    y2: int,
    hsv_background: NDArray[np.uint8],
) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]] | None:
    """
    Calculates four corners of object fitting bounding box by ignoring background
    with the specified color.

    :param x1,y1,x2,y2: bounding box

    Return four corners
    """
    height, width, _ = rgb_image.shape
    x1 = max(0, x1 - 5)
    y1 = max(0, y1 - 5)
    x2 = min(width - 1, x2 + 5)
    y2 = min(height - 1, y2 + 5)

    cropped_image = rgb_image[y1 : y2 + 1, x1 : x2 + 1]
    hsv_cropped_img = cv.cvtColor(cropped_image, cv.COLOR_RGB2HSV)
    background_mask = cv.inRange(hsv_cropped_img, hsv_background, hsv_background)

    cropped_image[background_mask > 0] = [128, 0, 128]

    cv.imwrite("cropped_mask.png", cropped_image)

    # ? Gets four corners of image
    gray_cropped_image = cv.cvtColor(cropped_image, cv.COLOR_RGB2GRAY)
    blurred_cropped_image = cv.GaussianBlur(gray_cropped_image, (5, 5), 0)

    edges = cv.Canny(blurred_cropped_image, 50, 150)
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)

        if len(approx) != 4:
            continue

        approx = approx.reshape((4, 2))
        centroid = np.mean(approx, axis=0)

        def angle_from_centroid(point):
            return np.arctan2(point[1] - centroid[1], point[0] - centroid[0])

        approx = sorted(approx, key=angle_from_centroid)  # Clockwise order

        for i in range(4):
            cv.line(
                cropped_image,
                tuple(approx[i]),
                tuple(approx[(i + 1) % 4]),
                (0, 255, 0),
                1,
            )

        for point in approx:
            cv.circle(cropped_image, tuple(point), 1, (0, 0, 255), -1)

        cv.imwrite("quad.png", cropped_image)

        print("Found quad!!")

        return tuple((int(corner[0]) + x1, int(corner[1]) + y1) for corner in approx)  # type: ignore[return-value]
    print("quad not found")

    return None

    # corners = cv.goodFeaturesToTrack(gray, 100, 0.01, 10)
    # corners = np.int0(corners)
    # + Radial sweep


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

            hsv_background = get_hsv_background(image, x1, y1, x2, y2)
            new_corners = None
            print(f"Encontrado background: {hsv_background}")
            if hsv_background is not None:
                new_corners = get_corners(image, x1, y1, x2, y2, hsv_background)
                if new_corners is not None:
                    print(f"Eram: {corners}, viraram {new_corners}")
                    # corners = new_corners

            camera_position = get_camera_focus_coordinate(
                robot_position=gps.get_position(), rotation_angle=imu.get_GPS_angle()
            )

            obj_points = np.array(
                (
                    (0.01, -0.01, 0),
                    (-0.01, -0.01, 0),
                    (-0.01, 0.01, 0),
                    (0.01, 0.01, 0),
                ),
                dtype=np.float32,
            )
            bounding_box = np.array(corners, dtype=np.float32).reshape((4, 1, 2))
            ret, rvecs, tvecs = cv.solvePnP(
                obj_points, bounding_box, camMatrix, distCoeff
            )
            rx, ry, rz = rvecs.ravel()
            print(ry / DEGREE_IN_RAD)
            print(
                f"Orientação da vítima: {cyclic_angle(ry - imu.get_rotation_angle()) / DEGREE_IN_RAD}"
            )
            tvec_dx, _dy, tvec_dz = tvecs.ravel()

            dx, dy = get_WT_coordinate_deltas(
                tvec_dx, tvec_dz, rotation_angle=imu.get_GPS_angle()
            )

            print(
                f"Encontrado {class_name}: {camera_position.x + dx}, {camera_position.y + dy}"
            )

            if new_corners is not None:
                bounding_box = np.array(new_corners, dtype=np.float32).reshape(
                    (4, 1, 2)
                )
                ret, rvecs, tvecs = cv.solvePnP(
                    obj_points, bounding_box, camMatrix, distCoeff
                )
                rx, ry, rz = rvecs.ravel()
                print(ry / DEGREE_IN_RAD)
                print(
                    f"Orientação 2 da vítima: {cyclic_angle(ry - imu.get_rotation_angle()) / DEGREE_IN_RAD}"
                )
                tvec_dx, _dy, tvec_dz = tvecs.ravel()

                dx, dy = get_WT_coordinate_deltas(
                    tvec_dx, tvec_dz, rotation_angle=imu.get_GPS_angle()
                )

                print(
                    f"Encontrado 2 {class_name}: {camera_position.x + dx}, {camera_position.y + dy}"
                )
    return wall_token
