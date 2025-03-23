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

from math import cos, sin

from ultralytics import YOLO  # type: ignore[import-untyped]

from devices import GPS, IMU, Camera, Lidar
from types_and_constants import DEGREE_IN_RAD, ROBOT_RADIUS, TILE_SIZE, WallToken

MODEL_PATH = r"/usr/local/controller/best.pt"
MIN_DIST_TO_SEND_WT = TILE_SIZE / 2 - 0.01
MIN_ACCURACY_TO_CONSIDER_WT = 0.85

model = YOLO(MODEL_PATH)


def classify_wall_token(
    camera: Camera,
    gps: GPS,
    lidar: Lidar,
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

            print(f"Acurácia: {conf * 100}%")

            if conf < MIN_ACCURACY_TO_CONSIDER_WT:
                continue

            side_angle = ((x1 + x2) / 2) / camera.width * camera.FOV - camera.FOV / 2
            angle = side_angle + imu.get_adjusted_angle()

            position = gps.get_position()
            x, y = position.x, position.y

            dist = (
                lidar.get_side_distance(side_angle, field_of_view=20 * DEGREE_IN_RAD)
                + ROBOT_RADIUS
            )

            dx = dist * cos(angle)
            dy = dist * sin(angle)

            if abs(x + dy) == float("inf") and abs(y + dy) == float("inf"):
                print(f"Ignorando por dar inf Lidar em: {side_angle / DEGREE_IN_RAD}")
                for ang, d in lidar.get_distances_by_side_angle().items():
                    print(f"{int(ang/DEGREE_IN_RAD)}: {d}")
                from time import sleep

                sleep(2)
                continue

            print(f"Encontrado {class_name} em: {x + dx}, {y + dy}")
    return wall_token
