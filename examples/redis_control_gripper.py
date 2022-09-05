import sys
sys.path.append('../src')

import redis
import pybullet as p
import msgpack
import time
import numpy as np
from math import sin
            #print(f"pointer positions: {pos_x}, {pos_y}")

from panda_gripper import Panda


duration = 1000
stepsize = 1e-3

robot = Panda(stepsize)
robot.setControlMode("position")

red = redis.StrictRedis('localhost', 6379)
sub = red.pubsub()
sub.subscribe('trajectory')

saferange_max_y = 0
saferange_min_y = 1.0

saferange_max_x = -0.5
saferange_min_x = 0.5

for i in range(int(duration/stepsize)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))

    pointer_pos = sub.get_message()
    
    if isinstance(pointer_pos, dict):
        
        try:
            data = msgpack.unpackb(pointer_pos['data'], raw=True)
            print(f"data: {data}")
            command = data[b'command'].decode('utf-8')
            if command == "reset":
                pass
            elif command == "track":
                pos_x = data[b'payload'][b'x']
                pos_y = data[b'payload'][b'y']
                xmin = data[b'payload'][b'xmin']
                ymin = data[b'payload'][b'ymin']
                xmax = data[b'payload'][b'xmax']
                ymax = data[b'payload'][b'ymax']
                
                ratio_x = (pos_x - xmin) / (xmax - xmin)
                ratio_y = (pos_y - ymin) / (ymax - ymin)

                endeffector_target_pos = [ratio_y * (saferange_max_y - saferange_min_y) + saferange_min_y, ratio_x * (saferange_max_x - saferange_min_x) + saferange_min_x, 0.1]

                target_pos = robot.solveInverseKinematics(endeffector_target_pos,[1,0,0,0])
                robot.setTargetPositions(target_pos)
            
        except Exception as e:
            print(f"Error occurred: {e} from message {pointer_pos}")
            continue

        
        
    robot.step()

    time.sleep(robot.stepsize)

