"""
WT size: 0.02 x 0.02
"""

import cv2 as cv
import numpy as np
from controller import Robot  # type: ignore

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

    from time import sleep

    sleep(10)
    bounding_box = [[x1, y1], [x2, y2]]
    cv.solvePnP(np.array([], bounding_box, dtype=np.float32))
