"""
WT size: 0.02 x 0.02
"""

from time import sleep

import cv2 as cv
import numpy as np
from controller import Robot  # type: ignore
from ultralytics import YOLO  # type: ignore[import-untyped]

MODEL_PATH = (
    r"/home/gustavo/Documentos/competicoes/olimpiadas/OBR/LARC/"
    r"rescue2024/github/auxiliar_codes/calibrate_camera/weights/best.pt"
)
model = YOLO(MODEL_PATH)

# Load camera calibration
with np.load("calibration.npz") as X:
    mtx, dist, _, _ = [X[i] for i in ("mtx", "dist", "rvecs", "tvecs")]

robot = Robot()

timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera2")
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()

while robot.step(timestep) != -1:
    camera_image = camera.getImage()
    image_argb = np.frombuffer(camera_image, dtype=np.uint8).reshape((height, width, 4))
    image = np.copy(image_argb[:, :, :3])

    results = model(image)

    class_names = model.names

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = class_names[class_id]

            cv.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"{class_name}: {conf:.2f}"
            cv.putText(
                image,
                text,
                (x1, y1 - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

            bounding_box = np.array([[x1, y1], [x2, y1], [x1, y2], [x2, y2]])
            obj_points = np.array(
                [[0, 0, 0], [0.02, 0, 0], [0, 0.02, 0], [0.02, 0.02, 0]],
                dtype=np.float32,
            )
            ret, rvecs, tvecs = cv.solvePnP(obj_points, bounding_box, mtx, dist)

    sleep(10)
