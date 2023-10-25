import numpy as np
import torch
import gym
import json
import argparse
import cv2
import os
import time
import math
from tensorboardX import SummaryWriter
import panda_gym

import nimblephysics as nimble

desired_goal = np.array((0.03, 0.01, 0.))

def create_world(time_step=0.01):
	# Create and configure world.
	world: nimble.simulation.World = nimble.simulation.World()
	world.setGravity([0, 0, -0.0])
	world.setTimeStep(time_step)

	return world

def render(path):

	world = create_world()
	# world.setDebug(True, False)

	floor = world.loadSkeleton("./urdf/table_real/table.urdf", np.array([0,0,0]), np.array([0,0,0]))
	floor.getBodyNodes()[0].setCollidable(False)

	franka = world.loadSkeleton("./urdf/franka_panda/panda_world.urdf", np.array([-0.39, 0.075, 0.0]), np.array([0,0,math.pi/2]))
	franka.setPositions([-1.6908033863401022, -0.3323558635047035, -0.07169693200038944, -2.667661599088951, -0.03265813189895577, 2.335847684122604, 0.620360233161526, 0.0085, 0.0085])
	franka.clampPositionsToLimits()
	for i in range(franka.getNumJoints()):
		_joint = franka.getJoint(i)
		_joint.setPositionLimitEnforced(True)
	for i in range(franka.getNumBodyNodes()):
		franka_node = franka.getBodyNode(i)
		franka_node.setFrictionCoeff(0)

	hole = world.loadSkeleton("./urdf/hole_big/hole.urdf", -np.array([0,0,0.01]) + desired_goal, np.array([0,0,0]))
	for i in range(hole.getNumBodyNodes()):
		hole_node = hole.getBodyNode(i)
		hole_node.setFrictionCoeff(0)

	gui = nimble.NimbleGUI(world)
	gui.serve(8080)
	gui.nativeAPI().renderWorld(world)

	states = torch.load(path)

	gui.loopStates(states)
	gui.blockWhileServing()
	gui.stopServing()



if __name__ == "__main__":

	

	render("peg_insertion_nimble.pt")
