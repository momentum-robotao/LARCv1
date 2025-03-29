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

import cv2 as cv
import numpy as np
from ultralytics import YOLO  # type: ignore[import-untyped]

from devices import GPS, Camera
from types_and_constants import TILE_SIZE, WallToken

MODEL_PATH = r"/usr/local/controller/best.pt"
CALIBRATION_PATH = r"/usr/local/controller/calibration.npz"
MIN_DIST_TO_SEND_WT = TILE_SIZE / 2 - 0.01
MIN_ACCURACY_TO_CONSIDER_WT = 0.85

model = YOLO(MODEL_PATH)

with np.load(CALIBRATION_PATH) as calibration_file:
    camMatrix, distCoeff, _, _ = [
        calibration_file[i] for i in ("camMatrix", "distCoeff", "rvecs", "tvecs")
    ]


def classify_wall_token(
    camera: Camera,
    gps: GPS,
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

            position = gps.get_position()
            x, y = position.x, position.y

            bounding_box = np.array(
                [[x1, y1], [x2, y1], [x1, y2], [x2, y2]], dtype=np.float32
            ).reshape((4, 1, 2))
            obj_points = np.array(
                [[0, 0, 0], [0.02, 0, 0], [0, 0.02, 0], [0.02, 0.02, 0]],
                dtype=np.float32,
            )
            ret, rvecs, tvecs = cv.solvePnP(
                obj_points, bounding_box, camMatrix, distCoeff
            )
            dy, _dz, dx = tvecs.ravel()

            print(dy, dx)

            print(f"Encontrado {class_name}: {x + dx}, {y + dy}")
    return wall_token
