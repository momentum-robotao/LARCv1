"""Use it instead of calling solve map"""
for ang in [0, 45, 90, 135, 180, 225, 270, 315, 360]:
    robot.motor.expected_angle = robot.imu.get_rotation_angle()
    robot.motor.expected_position = robot.gps.get_position()
    robot.motor.rotate("right", ang * DEGREE_IN_RAD, robot.imu)
    cnt = 0
    while robot.webots_robot.step(32) != -1:
        print("começa", robot.gps.get_position())
        robot.motor.move(
            "frontward", e testa backward
            robot.gps,
            robot.lidar,
            robot.color_sensor,
            robot.imu,
            robot.distance_sensor,
            robot.webots_robot,
        )
        print("termina", robot.gps.get_position())
        print("================")
        cnt += 1
        if cnt == 2:
            break
    print("------------------")
    robot.communicator.send_lack_of_progress()
    import time

    time.sleep(1)
