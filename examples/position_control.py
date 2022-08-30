import sys
sys.path.append('../src')

import pybullet as p
import time
import numpy as np
from math import sin

from panda_ball import Panda


duration = 10
stepsize = 1e-3

robot = Panda(stepsize)
robot.setControlMode("position")

vals = []

for i in range(int(duration/stepsize)):
    if i%1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))
 
    # if i%3000 == 0:
    #     robot.reset()
    #     robot.resetBall()
    #     robot.setControlMode("position")
    #     pos, vel = robot.getJointStates()
    #     target_pos = pos

    ball_pos, ball_ori = robot.getBallStates()
    print(f"ball position: {ball_pos}")
    vals.append(ball_pos)

    target_task_pos = ball_pos
    target_task_pos[2] += .5

    target_pos = robot.solveInverseKinematics(ball_pos,[1,0,0,0])
    robot.setTargetPositions(target_pos)

    robot.step()


    time.sleep(robot.stepsize)

val_np = np.array(vals)
print(f"shape: {val_np.shape}")
print(f"max x: {np.amax(val_np[:, 0])}, min x: {np.amin(val_np[:,0])}"), print(f"max y: {np.amax(val_np[:,1])}, min y: {np.amin(val_np[:,1])}"), print(f"max z: {np.amax(val_np[:,2])}, min z: {np.amin(val_np[:,2])}")
