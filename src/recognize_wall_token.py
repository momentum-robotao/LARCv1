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

from typing import Literal

import cv2
import numpy as np

from debugging import DebugInfo, System
from devices import Lidar
from types_and_constants import (
    DEBUG,
    DEGREE_IN_RAD,
    ROBOT_RADIUS,
    HazmatSign,
    Victim,
    WallToken,
)

MIN_DIST_TO_RECOGNIZE_WALL_TOKEN = 0.06  # TODO: ajustar, Nicolas colocou 0.08


img_height, img_width = 256, 320


def get_distance(angle_degree: int, lidar: Lidar) -> float:
    return (
        lidar.get_side_distance(
            angle_degree * DEGREE_IN_RAD, field_of_view=3 * DEGREE_IN_RAD
        )
        + ROBOT_RADIUS
    )


def get_dist_branco(raw_image, side: Literal["left", "right"], lidar: Lidar):
    """
    Logica: pega o maior contorno branco da imagem, e divide a imagem no meio
    se maior parte dos pixels brancos estiverem para a direita, por exemplo
    ele envia um sinal do lidar para o ponto DENTRO contorno mais distante do centro da imagem
    e retorna a distancia, no caso das outras 2 cores, se for menor que 0.08, ele retorna true
    """

    if side == "left":
        lidar_angle_base = 270
    else:
        lidar_angle_base = 90

    hsv_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 55, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv_image, lower_white, upper_white)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:  # TODO: função largest_countour
        largest_contour = max(contours, key=cv2.contourArea)
        center_x = raw_image.shape[1] // 2  # centro da imagem ('linha imaginaria')

        max_distance = 0
        farthest_point = None
        for point in largest_contour:
            x, y = point[0]
            distance = abs(x - center_x)
            if distance > max_distance:
                max_distance = distance
                farthest_point = (x, y)

        if farthest_point:
            farthest_x, farthest_y = farthest_point

            angle_offset = (
                farthest_x / raw_image.shape[1]
            ) * 20 - 10  # angulo relativo da camera
            lidar_angle = lidar_angle_base + angle_offset  # angulo para o lidar

            lidar_index = int(round(lidar_angle))

            distance_to_farthest_point = get_distance(lidar_index, lidar)

            return distance_to_farthest_point

    return 0


def check_organic_peroxide(  # TODO: juntar dist_branco, check_flamable
    raw_image, side: Literal["left", "right"], lidar: Lidar
) -> bool:
    if side == "left":
        lidar_angle_base = 270
    else:
        lidar_angle_base = 90

    hsv_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Contar a quantidade de pixels na máscara
    non_black_pixels = cv2.countNonZero(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        center_x = raw_image.shape[1] // 2

        # Encontrar o ponto mais distante do centro no eixo x
        max_distance = 0
        farthest_point = None
        for point in largest_contour:
            x, y = point[0]
            distance = abs(x - center_x)
            if distance > max_distance:
                max_distance = distance
                farthest_point = (x, y)

        if farthest_point:
            farthest_x, farthest_y = farthest_point

            # Calcular o ângulo relativo ao ponto mais distante do centro
            angle_offset = (farthest_x / raw_image.shape[1]) * 20 - 10
            lidar_angle = lidar_angle_base + angle_offset

            lidar_index = int(round(lidar_angle))

            distance_to_farthest_point = get_distance(lidar_index, lidar)

            if (
                distance_to_farthest_point < MIN_DIST_TO_RECOGNIZE_WALL_TOKEN
                and non_black_pixels > 60
            ):
                return True

    return False


def check_flamable_gas(raw_image, side: Literal["left", "right"], lidar: Lidar) -> bool:
    if side == "left":
        lidar_angle_base = 270
    else:
        lidar_angle_base = 90

    hsv_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 70, 50], dtype=np.uint8)
    upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red2 = np.array([170, 70, 50], dtype=np.uint8)
    upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Contar a quantidade de pixels na máscara
    non_black_pixels = cv2.countNonZero(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        center_x = raw_image.shape[1] // 2

        # Encontrar o ponto mais distante do centro no eixo x
        max_distance = 0
        farthest_point = None
        for point in largest_contour:
            x, y = point[0]
            distance = abs(x - center_x)
            if distance > max_distance:
                max_distance = distance
                farthest_point = (x, y)

        if farthest_point:
            farthest_x, farthest_y = farthest_point
            # Calcular o ângulo relativo ao ponto mais distante do centro
            angle_offset = (farthest_x / raw_image.shape[1]) * 20 - 10
            lidar_angle = lidar_angle_base + angle_offset

            lidar_index = int(round(lidar_angle))

            distance_to_farthest_point = get_distance(lidar_index, lidar)

            if (
                distance_to_farthest_point < MIN_DIST_TO_RECOGNIZE_WALL_TOKEN
                and non_black_pixels > 60
            ):
                return True

    return False


def check_robo_torto(lidar: Lidar, side: Literal["left", "right"]) -> bool:
    STEP = 20

    if side == "left":
        start_angle = 250
        end_angle = 290
    else:
        start_angle = 70
        end_angle = 110

    distancias = []
    min_distance = float("inf")
    min_angle = -1

    for angle in range(start_angle, end_angle + 1, STEP):
        distance = get_distance(angle, lidar)
        distancias.append(distance)

        if distance < min_distance:
            min_distance = distance
            min_angle = angle
    if min_angle == 90 or min_angle == 270:
        # ? robo ta certinho, mantem a margem
        return False
    else:
        # ? aumenta a margem do quadrado
        return True


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


def get_qty_preto(gray_image):
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


def get_image_information(raw_image, side: Literal["left", "right"], lidar: Lidar):
    gray_image = get_gray_image(raw_image)
    qty_preto = get_qty_preto(gray_image)
    dist_branco = get_dist_branco(raw_image, side, lidar)
    hazmat = verificar_HSU_ou_hazmat(raw_image)

    return (dist_branco, qty_preto, hazmat)


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

    resized_image = cv2.resize(cropped_image, (target_width, target_height))

    gray_resized = get_gray_image(resized_image)

    _, black_white_image = cv2.threshold(gray_resized, 127, 255, cv2.THRESH_BINARY)

    # TODO: função para essas masks
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
        quadrado,
        preto_vertical,
        preto_cima,
        preto_meio,
        preto_baixo,
        preto_vertical,
    )


def classify_H_S_U(margem, metrics):
    if not metrics:
        return None

    (
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


def find_bounding_box_around_white(raw_image):
    hsv_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 55, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv_image, lower_white, upper_white)

    # Encontrar a caixa ao redor da região branca
    x, y, w, h = cv2.boundingRect(mask)

    # Ajustar a caixa para ser um quadrado
    if w > h:
        y = max(0, y - (w - h) // 2)
        h = w
    else:
        x = max(0, x - (h - w) // 2)
        w = h

    # ? Verificar se a caixa delimitadora tem dimensões válidas
    if w <= 0 or h <= 0:
        return None

    return x, y, w, h


def H_S_U_perto(raw_image, target_width=320, target_height=256):
    """
    Basicamente igual ao outro reconhecedor de HSU. Diferença: não usa contorno branco.
    Pega direto a imagem e cria um quadrado, de modo a encaixar a maior quantidade de
    pixels brancos. Isso tira as bordas pretas, que não são vítimas.
    """
    if (bounding_box_around_white := find_bounding_box_around_white(raw_image)) is None:
        return 0
    x, y, w, h = bounding_box_around_white

    # Recortar a imagem para a caixa ajustada
    cropped_image = raw_image[y : y + h, x : x + w]
    resized_image = cv2.resize(cropped_image, (target_width, target_height))
    gray_resized = get_gray_image(resized_image)

    _, black_white_image = cv2.threshold(gray_resized, 127, 255, cv2.THRESH_BINARY)

    branco = cv2.countNonZero(black_white_image)

    lower_half = black_white_image[210:230, :]
    non_black_pixels_lower = cv2.countNonZero(lower_half)
    total_pixels_lower = lower_half.size
    preto_baixo = total_pixels_lower - non_black_pixels_lower

    mid_half = black_white_image[110:130, :]
    non_black_pixels_mid = cv2.countNonZero(mid_half)
    total_pixels_mid = mid_half.size
    preto_meio = total_pixels_mid - non_black_pixels_mid

    upper_half = black_white_image[26:46, :]
    non_black_pixels_upper = cv2.countNonZero(upper_half)
    total_pixels_upper = upper_half.size
    preto_cima = total_pixels_upper - non_black_pixels_upper

    vertical_slice = black_white_image[20:230, 118:138]
    non_black_pixels_vertical = cv2.countNonZero(vertical_slice)
    total_pixels_vertical = vertical_slice.size
    preto_vertical = (total_pixels_vertical - non_black_pixels_vertical) * 1.52
    preto_vertical = int(preto_vertical)

    if branco > 50000:
        # garantir que a vitima ta inteira na image, sujeito a mudanças
        if preto_vertical == min(
            preto_cima, preto_baixo, preto_vertical
        ) and preto_baixo == max(preto_cima, preto_meio, preto_baixo):
            return "U"
        elif preto_meio == max(
            preto_cima, preto_baixo, preto_meio
        ) and preto_vertical == min(preto_cima, preto_baixo, preto_vertical):
            return "H"
        elif preto_vertical == max(preto_cima, preto_baixo, preto_vertical):
            return "S"
        else:
            return 0


def classify_wall_token(
    image_information,
    raw_image,
    dist,
    debug_info: DebugInfo,
    margem,
    side: Literal["left", "right"],
    lidar: Lidar,
) -> WallToken | None:
    image_metrics = get_image_metrics(raw_image)

    if image_metrics:
        (
            quadrado,
            preto_vertical,
            preto_cima,
            preto_meio,
            preto_baixo,
            preto_vertical,
        ) = image_metrics

    (dist_branco, qty_preto, hazmat) = image_information
    wall_token: WallToken | None = None

    if check_organic_peroxide(raw_image, side, lidar):
        # esse range pode ser mais suave, pq a cor eh facil de reconhecer
        wall_token = HazmatSign.ORGANIC_PEROXIDE
    elif check_flamable_gas(raw_image, side, lidar):
        wall_token = HazmatSign.FLAMMABLE_GAS
    elif image_metrics is None:
        pass
    elif (
        hazmat >= 4
        and dist_branco < MIN_DIST_TO_RECOGNIZE_WALL_TOKEN
        and (preto_meio > 800 or qty_preto > 0)
    ):
        if qty_preto < 50:
            wall_token = HazmatSign.POISON
        elif qty_preto > 0:
            wall_token = HazmatSign.CORROSIVE
    elif (
        dist_branco < MIN_DIST_TO_RECOGNIZE_WALL_TOKEN
        and dist_branco > 0.053
        and qty_preto > 0
    ):
        # distancia ideal para reconhecimento + ou - 0.06
        if classify_H_S_U(margem, image_metrics) == "H":
            wall_token = Victim.HARMED
        elif classify_H_S_U(margem, image_metrics) == "S":
            wall_token = Victim.STABLE
        elif classify_H_S_U(margem, image_metrics) == "U":
            wall_token = Victim.UNHARMED
        else:
            print("TODO: há vítima, fazer estratégia pra 'encaixá-la'")
    elif dist_branco < 0.053 and qty_preto > 0:
        if H_S_U_perto(raw_image) == "H":
            wall_token = Victim.HARMED
        if H_S_U_perto(raw_image) == "S":
            wall_token = Victim.STABLE
        if H_S_U_perto(raw_image) == "U":
            wall_token = Victim.UNHARMED
    else:  # ? não tem vítima nenhuma na imagem
        pass
    if DEBUG:
        debug_info.send(f"{wall_token=}", System.wall_token_classification)
    return wall_token


# TODO: world1.wbt pegar vítima entre dfs's


def reconhece_lado(
    camera,
    debug_info,
    side: Literal["left", "right"],
    lidar: Lidar,
    rotating: bool = False,
):
    # ? FOV=20: mudei isso aqui pra pegar o quarter tile, ent so de ter parede na camera
    # ele ja vai reconhecer
    dist = (
        lidar.get_side_distance(side, field_of_view=20 * DEGREE_IN_RAD, use_min=True)
        + ROBOT_RADIUS
    )
    if dist >= MIN_DIST_TO_RECOGNIZE_WALL_TOKEN:
        if DEBUG:
            debug_info.send(
                "No wall to recognize wall token", System.wall_token_recognition
            )
        return

    raw_image = camera.getImage()
    raw_image = np.frombuffer(raw_image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

    image_information = get_image_information(raw_image, side, lidar)
    # TODO: separar em reconhece perto e reconhece longe e função que identifica caso
    if check_robo_torto(lidar, side):
        """
        Se a vitima (cropped image) estiver inteira na imagem, a imagem vai ser um quadrado, mas
        quando o robo ta angulado em relação a parede, a imagem da vitima vai ser tipo um trapezio
        (ponto de fuga no fundo da imagem), entao eu fiz um metodo pra aumentar a margem do
        quadrado, se o robo estiver angulado como eu não tenho o docker pra rodar o codigo, tem que
        ir mudando essa marge, mas deve ta meio bom <= TODO
        """
        margem = 0.5
        if rotating:
            margem = 3.5
    else:
        margem = 0.2
    return classify_wall_token(
        image_information, raw_image, dist, debug_info, margem, side, lidar
    )
