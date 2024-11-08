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

MIN_DIST_TO_RECOGNIZE_WALL_TOKEN = 0.06


img_height, img_width = 256, 320


def get_gray_image(raw_image):
    """Returns image of camera in gray scale."""
    gray_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
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


def verificar_HSU_ou_hazmat(raw_image):
    polygon_count = 0
    res = cv2.resize(raw_image, (img_width, img_height))
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


def get_image_information(raw_image):
    gray_image = get_gray_image(raw_image)
    preto = quant_preto(gray_image)
    branco = quant_branco(gray_image)
    polygon_count = 0
    hazmat = verificar_HSU_ou_hazmat(raw_image)

    image = raw_image[:, :, :3]
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
    return (branco, preto, mask_yellow, mask_red, hazmat, polygon_count)


def get_image_metrics(raw_image, target_width=320, target_height=256):
    gray_image = get_gray_image(raw_image)

    _, binary = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    cropped_image = raw_image[y : y + h, x : x + w]
    shape = cropped_image.shape
    altura = shape[0]
    largura = shape[1]
    quadrado = altura / largura
    margem = 0.45

    resized_image = cv2.resize(cropped_image, (target_width, target_height))

    gray_resized = get_gray_image(resized_image)

    _, black_white_image = cv2.threshold(gray_resized, 127, 255, cv2.THRESH_BINARY)

    lower_half = black_white_image[190:210, :]
    non_black_pixels_lower = cv2.countNonZero(lower_half)
    total_pixels_lower = lower_half.size
    preto_baixo = total_pixels_lower - non_black_pixels_lower

    mid_half = black_white_image[110:130, :]
    non_black_pixels_mid = cv2.countNonZero(mid_half)
    total_pixels_mid = mid_half.size
    preto_meio = total_pixels_mid - non_black_pixels_mid

    upper_half = black_white_image[55:75, :]
    non_black_pixels_upper = cv2.countNonZero(upper_half)
    total_pixels_upper = upper_half.size
    preto_cima = total_pixels_upper - non_black_pixels_upper

    vertical_slice = black_white_image[20:230, 118:138]
    non_black_pixels_vertical = cv2.countNonZero(vertical_slice)
    total_pixels_vertical = vertical_slice.size
    preto_vertical = (total_pixels_vertical - non_black_pixels_vertical) * 1.52
    preto_vertical = int(preto_vertical)

    return (
        margem,
        quadrado,
        preto_vertical,
        preto_cima,
        preto_meio,
        preto_baixo,
        preto_vertical,
    )


def classify_H_S_U(metrics):
    if not metrics:
        return None

    (
        margem,
        quadrado,
        preto_vertical,
        preto_cima,
        preto_meio,
        preto_baixo,
        preto_vertical,
    ) = metrics

    if (1 - margem) <= quadrado <= (1 + margem):
        if preto_vertical == min(
            preto_cima, preto_meio, preto_baixo, preto_vertical
        ) and preto_baixo == max(preto_cima, preto_meio, preto_baixo, preto_vertical):
            return "U"
        elif preto_meio == max(
            preto_cima, preto_baixo, preto_meio
        ) and preto_vertical == min(preto_cima, preto_baixo, preto_vertical):
            return "H"
        elif preto_vertical == max(preto_cima, preto_baixo, preto_meio, preto_vertical):
            return "S"
        else:
            return 0


def classify_wall_token(
    image_information,
    image_metrics,
    dist,
    debug_info: DebugInfo,
) -> WallToken | None:
    (
        branco,
        preto,
        mask_yellow,
        mask_red,
        hazmat,
        polygon_count,
    ) = image_information
    print(dist)
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
    elif branco > 0 and preto > 0 and dist > 0.015 and dist < 0.068:
        if classify_H_S_U(image_metrics) == "H":
            wall_token = Victim.HARMED
        elif classify_H_S_U(image_metrics) == "S":
            wall_token = Victim.STABLE
        elif classify_H_S_U(image_metrics) == "U":
            wall_token = Victim.UNHARMED
        else:
            # TODO: tem vitima, mas para reconhecer corretamente ela deve estar inteira na imagem
            # faria uma estrategia para "encaixar" a vitima inteira
            print("encaixar")
    elif (
        branco > 0 and preto > 0 and dist < 0.015
    ):  # TODO: testar se dfs é nessa dist => senão ajusta nesse caso
        # TODO: muito perto
        print("aproximar")
    if DEBUG:
        debug_info.send(f"{wall_token=}", System.wall_token_classification)
    return wall_token


# TODO: world1.wbt pegar vítima entre dfs's


def reconhece_lado(dist, camera, debug_info):
    if dist > MIN_DIST_TO_RECOGNIZE_WALL_TOKEN:
        debug_info.send(
            "No wall to recognize wall token", System.wall_token_recognition
        )
        return

    raw_image = camera.getImage()
    raw_image = np.frombuffer(raw_image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

    image_information = get_image_information(raw_image)
    image_metrics = get_image_metrics(raw_image)

    return classify_wall_token(image_information, image_metrics, dist, debug_info)


def recognize_wall_token(robot: Robot, debug_info: DebugInfo) -> None:
    for wall_token in [
        reconhece_lado(
            robot.lidar.get_side_distance("left"), robot.camera._left_camera, debug_info
        ),
        reconhece_lado(
            robot.lidar.get_side_distance("right"),
            robot.camera._right_camera,
            debug_info,
        ),
    ]:
        if wall_token:
            robot.motor.stop()
            delay(robot.webots_robot, debug_info, 1300)
            robot.communicator.send_wall_token_information(
                robot.gps.get_position(), wall_token
            )
