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

import nimblephysics as nimble

import ipdb

robot_init_state = [ 0.07129488 ,-0.82724565, -2.68480527, -1.54026022 , 0.20306757,  1.29076029,
 -0.25689644,  0.0085 ,     0.0085    ]

def create_world(time_step=0.01):
	# Create and configure world.
	world: nimble.simulation.World = nimble.simulation.World()
	world.setGravity([0, 0, -0.0])
	world.setTimeStep(time_step)

	return world

def render(path):
	# ipdb.set_trace()
	world = create_world()

	table = world.loadSkeleton("./urdf/table_real/table.urdf", np.array([0, 0, -0.002]), np.array([0, 0, 0]))
	table.getBodyNodes()[0].setCollidable(False)
	
	franka = world.loadSkeleton("./urdf/franka_panda/panda_with_spatula.urdf",
								np.array([-0.39, 0.075, 0.0]),  # np.array([-0.6, 0.075, 0.0]),
								np.array([0, 0, math.pi / 2]))

	franka.setPositions(robot_init_state)

	for i in range(franka.getNumBodyNodes()):
		franka_node = franka.getBodyNode(i)
		franka_node.setFrictionCoeff(2.)
		franka_node.setGravityMode(False)
		franka_node.setCollidable(False)
	franka.getBodyNodes("spatula_link")[0].setCollidable(True)



	pan = world.loadSkeleton("./urdf/pan3/pan3.urdf", np.array([-0.1, -0.07, 0.025]), np.array([0, 0, 0]))
	pan.getBodyNodes()[0].setCollidable(False)


	pancake = world.loadSkeleton("./urdf/pancake/pancake2.urdf", np.array((-0.1, -0.09, 0.065425)), np.array([0, 0, math.pi / 2])) 

	for i in range(pancake.getNumBodyNodes()):
		pancake_node = pancake.getBodyNode(i)
		pancake_node.setFrictionCoeff(1.)

	gui = nimble.NimbleGUI(world)
	gui.serve(8090)
	gui.nativeAPI().renderWorld(world)

	states = torch.load(path)

	gui.loopStates(states)
	gui.blockWhileServing()
	gui.stopServing()



if __name__ == "__main__":

	render('./data/pancake_lift_nimble.pt')
