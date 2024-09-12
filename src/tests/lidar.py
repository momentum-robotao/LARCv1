from controller import Robot, Lidar # Step 1: Import Lidar
from time import sleep
robot = Robot()
lidar = robot.getDevice("lidar") # Step 2: Retrieve the sensor, named "lidar", from the robot. Note that the sensor name may differ between robots.

timestep = 32

lidar.enable(timestep) # Step 3: Enable the sensor, using the timestep as the update rate
ok = True
while robot.step(timestep) != -1:
    if ok:
        rangeImage = lidar.getLayerRangeImage(0) # Step 4: Retrieve the range image
        
        # Print the first 10 values of the range image.
        # The range image stores the distances from left to right, from first to last layer
        for i, dist in enumerate(rangeImage):
            print(f"dist[{i}] = {dist}")
            #sleep(0.5)
        ok = False
