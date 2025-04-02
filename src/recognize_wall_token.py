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

import itertools
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


def find_farthest_four(points: np.ndarray) -> np.ndarray:
    """
    Find four farthest points from an approxPolyDP result.

    :param points: NumPy array of shape (N, 1, 2), result of cv2.approxPolyDP.
    :return: NumPy array of shape (4, 2) containing the four farthest points.
    """
    points = points[:, 0]  # Reshape from (N, 1, 2) to (N, 2)

    # Compute pairwise distances
    max_dist = 0.0
    farthest_pair: tuple[np.ndarray | None, np.ndarray | None] = (None, None)
    for p1, p2 in itertools.combinations(points, 2):
        dist = float(np.linalg.norm(p1 - p2))
        if dist > max_dist:
            max_dist = dist
            farthest_pair = (p1, p2)

    remaining_points = np.array(
        [
            p
            for p in points
            if not np.array_equal(p, farthest_pair[0])
            and not np.array_equal(p, farthest_pair[1])
        ]
    )
    # TODO: filter min expected dist to avoid rectangles inside WTs pattern

    # Find the next two points that maximize the area
    max_area = 0.0
    best_quad: np.ndarray = np.zeros((4, 2), dtype=np.int32)
    for p3, p4 in itertools.combinations(remaining_points, 2):
        quad = np.array([farthest_pair[0], farthest_pair[1], p3, p4])
        area = cv.contourArea(quad)
        if area > max_area:
            max_area = area
            best_quad = quad

    return best_quad


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
    print(
        f"Convex Hull: {cv.convexHull(np.array(pts).reshape((len(pts), 2)), clockwise=True)}"
    )
    masked_cropped_image[background_mask > 0] = [255, 255, 255]
    masked_cropped_image[background_mask == 0] = [0, 0, 0]

    cv.imwrite("cropped_mask.png", masked_cropped_image)

    edges = cv.Canny(masked_cropped_image, 50, 150)
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # TODO: test findCorners
    # TODO: pegar 4 pontos maximizando área
    # TODO: pintar interior de outra cor também

    print(cropped_image)

    for contour in contours:
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)

        last = approx[-1].ravel()
        for pt_ in approx:
            pt = pt_.copy().ravel()
            cv.circle(cropped_image, tuple(pt), 1, (0, 0, 255), -1)
            cv.line(
                cropped_image,
                tuple(last),
                tuple(pt),
                (0, 255, 0),
                1,
            )
        cv.imwrite("found.png", cropped_image)

        approx = find_farthest_four(approx)

        if len(approx) != 4:
            continue

        approx = approx.reshape((4, 2))
        centroid = np.mean(approx, axis=0)

        def angle_from_centroid(point):
            return np.arctan2(point[1] - centroid[1], point[0] - centroid[0])

        approx = sorted(approx, key=angle_from_centroid)  # Clockwise order

        for i in range(4):
            cv.line(
                masked_cropped_image,
                tuple(approx[i]),
                tuple(approx[(i + 1) % 4]),
                (0, 255, 0),
                1,
            )

        for point in approx:
            cv.circle(masked_cropped_image, tuple(point), 1, (0, 0, 255), -1)

        cv.imwrite("quad.png", masked_cropped_image)

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
