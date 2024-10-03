from controller import Robot, Lidar # Step 1: Import Lidar
import numpy as np
from time import sleep
robot = Robot()
lidar = robot.getDevice("lidar") # Step 2: Retrieve the sensor, named "lidar", from the robot. Note that the sensor name may differ between robots.

timestep = int(robot.getBasicTimeStep())

#https://v23.erebus.rcj.cloud/docs/tutorials/sensors/lidar/

def get_value(angle, lidar) -> float:
    rangeImage = lidar.getRangeImage() # Step 4: Retrieve the range image
    n = (angle*512)//360
    print()
    dist = 0
    count = 0
    for i in range(4):
        if rangeImage[n+ i*512] != float('inf') : 
            dist += rangeImage[n+ i*512]
            count +=1

    dist = dist/count

    return dist
    
lidar.enable(timestep) # Step 3: Enable the sensor, using the timestep as the update rate
ok = True
x = []
y = []
while robot.step(timestep) != -1:
    if ok:
        print(f"this is the max range {lidar.getMaxRange()} ")
        print(f"this is the min range {lidar.getMinRange()} ")

        for i in range(360): 
            print(f"dist angle {i} is {get_value(i, lidar)}")

        ''' 
        rangeImage = lidar.getRangeImage() # Step 4: Retrieve the range image

    
        
        # Print the first 10 values of the range image.
        # The range image stores the distances from left to right, from first to last layer
        print(f"lidar: {np.asarray(rangeImage).shape} ")
        for i, dist in enumerate(rangeImage):
            print(f"dist[{i}] = {dist}")
            #sleep(0.5)
        '''
        #ok = False
