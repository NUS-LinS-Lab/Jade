import os
import math
import time
import argparse
import torch
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_data as pd
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
# p.configureDebugVisualizer(p.COV_ENABLE_X_AXIS_UP, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 0)
# p.configureDebugVisualizer(p.COV_ENABLE_Z_AXIS_UP, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setTimeStep(0.01)
p.setGravity(0,0,0)
startPos = [0.0,0.0,0.0]
startPos1 = [0.03, 0.01, -0.01]
startPos2 = [-0.39, 0.075, 0.0]
startOrientation = p.getQuaternionFromEuler([0,0,math.pi/2])
startOrientation1 = p.getQuaternionFromEuler([0,0,0])
urdfFlags = p.URDF_USE_SELF_COLLISION

color_robot = [0, 255, 0, 255]

planeId = p.loadURDF("./urdf/table_real/table.urdf",startPos)

cube1Id = p.loadURDF("./urdf/hole_big/hole.urdf",startPos1, startOrientation1, flags=urdfFlags)

robotId = p.loadURDF("./urdf/franka_panda/panda_world.urdf", startPos2, startOrientation)

p.changeVisualShape(robotId, 12, rgbaColor=[i / 255.0 for i in color_robot])

p.resetDebugVisualizerCamera(0.3,0,-15,[0,0,0.15])

log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./videos/peg_nimble.mp4")

Joint_List = [1, 2, 3, 4, 5, 6, 7, 10, 11]
Joint_Init_State = [-1.6908033863401022, -0.3323558635047035, -0.07169693200038944, -2.667661599088951, -0.03265813189895577, 2.335847684122604, 0.620360233161526, 0.0085, 0.0085]

num_joints = p.getNumJoints(robotId)

for i in range(len(Joint_List)):
    p.resetJointState(bodyUniqueId=robotId, jointIndex=Joint_List[i], targetValue=Joint_Init_State[i])
    
states = torch.load("./data/peg_insertion_nimble.pt")

forces = [500.0 for _ in range(len(Joint_List))]

DURATION = len(states)
print("steps: ", DURATION)
for i in range(DURATION):
    print(i)
    q = states[i][0:9].tolist()
    qd = states[i][9:18].tolist()
    p.stepSimulation()
    # input()
    # p.setJointMotorControlArray(bodyUniqueId=robotId, jointIndices=Joint_List, controlMode=p.POSITION_CONTROL, targetPositions=q, targetVelocities=qd, forces=forces)
    
    for i in range(len(Joint_List)):
        p.resetJointState(bodyUniqueId=robotId, jointIndex=Joint_List[i], targetValue=q[i])
    time.sleep(1.)
    
p.stopStateLogging(log_id)


