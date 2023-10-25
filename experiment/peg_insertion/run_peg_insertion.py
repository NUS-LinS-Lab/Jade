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

def create_world(time_step=0.01):
	# Create and configure world.
	world: nimble.simulation.World = nimble.simulation.World()
	world.setGravity([0, 0, -0.0])
	world.setTimeStep(time_step)

	return world

def model_base_assemble(desired_goal, move_num_steps=10, assemble_num_steps=5):
	# Set up Nimble
	flag = False
	num_timesteps = move_num_steps + assemble_num_steps

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

	if args.vis:
		gui = nimble.NimbleGUI(world)
		gui.serve(8080)
		gui.nativeAPI().renderWorld(world)

	ikMap = nimble.neural.IKMapping(world)
	handNode = franka.getBodyNode("panda_grasptarget")
	ikMap.addSpatialBodyNode(handNode)
	actions_robot = torch.zeros((num_timesteps, 7), requires_grad=True)
	optimizer = torch.optim.Adam([{'params': actions_robot}], lr=1.0)
	#actions_other = torch.zeros((num_timesteps, world.getNumDofs() - 7), requires_grad=False)

	init_state = torch.Tensor(world.getState())
	print("Initial state:", init_state)
	init_hand_pos = nimble.map_to_pos(world, ikMap, init_state)
	target_pos = torch.Tensor(desired_goal)
	print("Target Position:", target_pos)

	for k in range(3000):
		state = init_state
		states = [state.detach()]
		hand_poses = [nimble.map_to_pos(world, ikMap, state).detach()]
		for i in range(move_num_steps):
			action = torch.concat([actions_robot[i], torch.zeros(2,)], -1)
			state = nimble.timestep(world, state, action)
			state[7:9] = 0.009
			#print(state)
			states.append(state.detach())
			hand_poses.append(nimble.map_to_pos(world, ikMap, state).detach())
		if args.vis:
			gui.loopStates(states)
		hand_pos = nimble.map_to_pos(world, ikMap, state)
		print(hand_pos)
		loss_grasp = ((hand_pos[3:] - torch.Tensor([0.005, 0.0, 0.08]) - target_pos)**2 * torch.tensor([10.0, 10.0, 1.0])).mean() + 10 * ((torch.abs(hand_pos[:3]) - torch.Tensor([3.14,0.0,0.0]))**2).mean()
		loss_stop = (state[9:]**2).mean()
		loss = loss_grasp + 0.01*loss_stop
		
		optimizer.zero_grad()
		loss.backward()
		optimizer.step()
		print("move", k, loss,)
		# input()
		if loss < 5e-5:
			flag = True
			break

	if not flag:
		return None, flag
	flag = False

	move_state = states
	move_hand_poses = hand_poses

	# state = init_state
	# move_state = [state.detach()]
	# move_hand_poses = [nimble.map_to_pos(world, ikMap, state).detach()]

	for k in range(50000):
		state = move_state[-1]
		state[9:] = 0
		states = move_state.copy()
		hand_poses = move_hand_poses.copy()

		for i in range(assemble_num_steps):
			action = torch.concat([actions_robot[move_num_steps + i], torch.zeros(2,)], -1)
			state = nimble.timestep(world, state, action)
			state[7:9] = 0.009
			states.append(state.detach())
			hand_poses.append(nimble.map_to_pos(world, ikMap, state).detach())
			# hand_poses.append(nimble.map_to_pos(world, ikMap, state))
		if args.vis:
			gui.loopStates(states)

		hand_pos = nimble.map_to_pos(world, ikMap, state)
		loss_grasp = ((hand_pos[3:] - torch.Tensor([0.0, 0.0, 0.04]) - target_pos)**2 * torch.tensor([2.0, 2.0, 1.0])).mean()  + ((torch.abs(hand_pos[:3]) - torch.Tensor([3.14,0.0,0.0]))**2).mean()
		loss_stop = (state[9:]**2).mean()
		loss = loss_grasp + 0.01*loss_stop

		# loss = 0
		# for hand_pos in hand_poses:
		# 	loss += ((hand_pos[3:] - torch.Tensor([0, 0.0, 0.04]) - target_pos)**2 * torch.tensor([2.0, 2.0, 1.0])).mean()  + ((torch.abs(hand_pos[:3]) - torch.Tensor([3.14,0.0,0.0]))**2).mean()
		
		# hand_pos_final = nimble.map_to_pos(world, ikMap, state).detach()
		# loss_grasp = ((hand_pos_final[3:] - torch.Tensor([0, 0.0, 0.04]) - target_pos)**2 * torch.tensor([2.0, 2.0, 1.0])).mean()  + ((torch.abs(hand_pos_final[:3]) - torch.Tensor([3.14,0.0,0.0]))**2).mean()

		if args.vis:
			gui.loopStates(states)
		optimizer.zero_grad()
		loss.backward()
		optimizer.step()
		print("assemble", k, loss_grasp)
		if args.vis:
			gui.loopStates(states)
		if torch.isnan(actions_robot.grad).any():
			input()
		if loss < 5e-5:
			flag = True
			break
	
	_states = np.concatenate(states).reshape(-1,18)
	joint_actions = _states[1:,:7] - _states[:num_timesteps,:7]

	torch.save(states, './data/peg_insertion_nimble.pt')
	if args.vis:
		gui.loopStates(states)
		gui.blockWhileServing()
		gui.stopServing()

	return joint_actions, flag


if __name__ == "__main__":
	start_time = time.time()
	parser = argparse.ArgumentParser()
	parser.add_argument("--env", default="PandaAssembleJointsDense-v2")          # OpenAI gym environment name
	parser.add_argument("--discount", default=0.9)  
	parser.add_argument("--seed", default=970101, type=int)
	parser.add_argument("--move_step", default=30, type=int)
	parser.add_argument("--assemble_step", default=10, type=int)                 
	parser.add_argument("--data_dir", default="./data/demo") 
	parser.add_argument("--rand_cam", action="store_true")
	parser.add_argument("--camera_mode", default="2dof")
	parser.add_argument("--vis", action="store_true") 
	args = parser.parse_args()

	mode = 'train'
	num = 1
	move_num_steps = args.move_step
	assemble_num_steps = args.assemble_step
	num_timesteps = move_num_steps + assemble_num_steps

	desired_goal = np.array((0.03, 0.01, 0.))
	joint_actions, success = model_base_assemble(desired_goal, move_num_steps, assemble_num_steps)
	if success:
		print('success')
