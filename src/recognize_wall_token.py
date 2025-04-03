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

from math import acos, asin, atan, atan2, cos, sin, sqrt

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


def cross(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    return x1 * y2 - x2 * y1


def subtract(pt1, pt2):
    pt1[0] -= pt2[0]
    pt1[1] -= pt2[1]
    return pt1


def sz(pt):
    return sqrt(pt[0] * pt[0] + pt[1] * pt[1])


def dot(pt1, pt2):
    return pt1[0] * pt2[0] + pt1[1] * pt2[1]


def remove_pts_by_angle(pts, ang=130 * DEGREE_IN_RAD):
    n = len(pts)
    result = []
    for i in range(n):
        pt_left = pts[(i - 1 + n) % n].copy()
        pt = pts[i].copy()
        pt_right = pts[(i + 1) % n].copy()

        print(pt_left, pt, pt_right, end=": ")
        left = subtract(pt_left, pt)
        right = subtract(pt_right, pt)
        sin_angle = round(cross(left, right) / sz(left) / sz(right), 4)
        cos_angle = round(dot(left, right) / sz(left) / sz(right), 4)
        angle = atan2(sin_angle, cos_angle)
        print(angle / DEGREE_IN_RAD)
        if angle <= ang:
            result.append(pts[i].copy())
    return result


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

    masked_cropped_image = cropped_image.copy()
    pts = []
    for x in range(x2 - x1 + 1):
        for y in range(y2 - y1 + 1):
            if background_mask[y, x] == 0:
                pts.append((x, y))
    convex_hull = cv.convexHull(np.array(pts).reshape((len(pts), 2)), clockwise=True)

    convex_hull = convex_hull.reshape((len(convex_hull), 2))

    print(f"Convex hull: {convex_hull}")
    if len(convex_hull) >= 3:
        convex_hull = remove_pts_by_angle(convex_hull)
        print(f"Novo ch: {convex_hull}")
        for i in range(len(convex_hull)):
            cv.line(
                masked_cropped_image,
                tuple(convex_hull[i]),
                tuple(convex_hull[(i + 1) % len(convex_hull)]),
                (0, 255, 0),
                1,
            )

        for point in convex_hull:
            cv.circle(masked_cropped_image, tuple(point), 1, (0, 0, 255), -1)

        cv.imwrite("quad.png", masked_cropped_image)

    if len(convex_hull) != 4:
        print("quad not found")
        return None

    print("quad found")

    return tuple((x, y) for x, y in convex_hull)


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
                print(imu.get_rotation_angle() / DEGREE_IN_RAD)
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
