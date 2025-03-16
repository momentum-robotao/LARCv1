"""
Ideia:
- enquadrar um quadrado exatamente na câmera e ver no LIDAR
- ou alinha ele no bloco e gira até não ter mais na câmera, girou FOV **

Último: 0.5708 rad
"""

from controller import Robot  # type: ignore

robot = Robot()

timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera2")
camera.enable(timestep)

imu = robot.getDevice("inertial_unit")
imu.enable(timestep)

img = None

while robot.step(timestep) != -1:
    rotation_angle = imu.getRollPitchYaw()[2]
    print(rotation_angle)
    try:
        img = camera.getImage()
        print(camera.getFov())
    except Exception as exc:
        print(exc)

print(img)
