"""
Regras importantes:
    Próximo
    "For successful wall token identification (TI) [40], the center of the robot must be
    equal to or less than half a tile distance from the location of the wall token
    when the robot indicates a wall token has been identified."

    Parado
    "To identify a wall token, the robot must stop at each one for 1 second.
    After 1 second, it must send a command to the game manager with the type
    of the wall token in a platform-specific format"
"""

import cv2
import numpy as np

from debugging import DebugInfo, System
from helpers import delay
from robot import Robot
from types_and_constants import DEBUG, HazmatSign, Victim, WallToken

MIN_DIST_TO_RECOGNIZE_WALL_TOKEN = 0.4


img_height, img_width = 256, 320


def get_gray_image(camera):
    """Returns image of camera in gray scale."""
    image = camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray_image


def quant_branco(gray_image):
    quant = 0
    for linhas in gray_image:
        for pixels in linhas:
            if pixels > 150:
                quant += 1
    return quant


def quant_preto(gray_image):
    non_black_pixels = cv2.countNonZero(gray_image)
    total_pixels = gray_image.size
    black_pixels = total_pixels - non_black_pixels
    return black_pixels


def verificar_HSU_ou_hazmat(image):
    polygon_count = 0
    res = cv2.resize(image, (img_width, img_height))
    hsv_image = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

    # Define the lower and upper threshold values for black in HSV
    lower_black = np.array([0, 0, 0], dtype=np.uint8)
    upper_black = np.array([180, 255, 185], dtype=np.uint8)
    mask = cv2.inRange(hsv_image, lower_black, upper_black)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.002 * cv2.arcLength(cnt, True), True)
        if len(approx) >= 3:
            polygon_count += 1

    return polygon_count


def count_and_display_polygons(robot: Robot, debug_info: DebugInfo):
    image = robot.camera._left_camera.getImage()
    image = np.frombuffer(image, np.uint8).reshape(
        (robot.camera._left_camera.getHeight(), robot.camera._left_camera.getWidth(), 4)
    )

    gray_image = get_gray_image(robot.camera._left_camera)
    preto = quant_preto(gray_image)
    branco = quant_branco(gray_image)
    polygon_count = 0
    hazmat = verificar_HSU_ou_hazmat(image)

    image = image[:, :, :3]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.resize(gray, (img_width, img_height))
    blurred_image = cv2.GaussianBlur(res, (5, 5), 0)

    edges = cv2.Canny(blurred_image, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) >= 3:
            polygon_count += 1

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    return branco, preto, mask_yellow, mask_red, hazmat, polygon_count


def classify_wall_token(
    branco,
    preto,
    mask_yellow,
    mask_red,
    hazmat,
    polygon_count,
    debug_info: DebugInfo,
) -> WallToken | None:
    wall_token: WallToken | None = None
    if np.any(mask_yellow > 0):
        wall_token = HazmatSign.ORGANIC_PEROXIDE
    elif np.any(mask_red > 0):
        wall_token = HazmatSign.FLAMMABLE_GAS
    elif hazmat >= 4:
        if preto < 10:
            wall_token = HazmatSign.POISON
        else:
            wall_token = HazmatSign.CORROSIVE
    elif branco > 300:
        if polygon_count <= 20:
            wall_token = Victim.HARMED
        elif polygon_count <= 25:
            wall_token = Victim.UNHARMED
        else:
            wall_token = Victim.STABLE
    if DEBUG:
        debug_info.send(f"{wall_token=}", System.wall_token_classification)
    return wall_token


def recognize_wall_token(robot: Robot, debug_info: DebugInfo) -> None:
    if robot.lidar.get_side_distance("left") > MIN_DIST_TO_RECOGNIZE_WALL_TOKEN:
        debug_info.send(
            "No left wall to recognize wall token", System.wall_token_recognition
        )
        return

    branco, preto, mask_yellow, mask_red, hazmat, polygon_count = (
        count_and_display_polygons(robot, debug_info)
    )
    wall_token = classify_wall_token(
        branco, preto, mask_yellow, mask_red, hazmat, polygon_count, debug_info
    )
    if wall_token:
        robot.motor.stop()
        delay(robot.webots_robot, debug_info, 1300)
        robot.communicator.send_wall_token_information(
            robot.gps.get_position(), wall_token
        )
