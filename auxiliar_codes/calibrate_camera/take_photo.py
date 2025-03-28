from controller import Robot  # type: ignore

robot = Robot()

timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera2")
camera.enable(timestep)

from random import randint

x = randint(0, 100000000)
while robot.step(timestep) != -1:
    from time import sleep

    sleep(3)
    camera.saveImage(f"image{x}.png", 256)
    x += 1
