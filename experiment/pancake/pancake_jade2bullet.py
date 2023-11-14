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
import pybullet as p
import pybullet_data

robot_init_state = [ 0.07129488 ,-0.82724565, -2.68480527, -1.54026022 , 0.20306757,  1.29076029,
 -0.25689644,  0.0085 ,     0.0085    ]


def create_world(time_step=0.01):
	# Create and configure world.
	world: nimble.simulation.World = nimble.simulation.World()
	world.setGravity([0, 0, -10.0])
	world.setTimeStep(time_step)

	return world

def model_base_assemble(desired_goal, pancake_init_pos, num_timesteps=15):
	# Set up Nimble
	flag = False

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

	# start init bullet
	physicsClient = p.connect(p.GUI)
	p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	p.setTimeStep(0.01)
	p.setGravity(0, 0, 0)

	pancake_init_pos = np.array([-0.1, -0.09, 0.065425])

	urdfFlags = p.URDF_USE_SELF_COLLISION

	color_robot = [0, 255, 0, 255]

	tableId = p.loadURDF("./urdf/table_real/table.urdf",
						basePosition=np.array([0, 0, -0.002]),
						baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
	robotId = p.loadURDF("./urdf/franka_panda/panda_with_spatula.urdf",
						basePosition=np.array([-0.39, 0.075, 0.0]),
						baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi / 2]))
	panId = p.loadURDF("./urdf/pan3/pan3.urdf",
					basePosition=np.array([-0.1, -0.07, 0.025]),
					baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
	pancakeId = p.loadURDF("./urdf/pancake/pancake2_bullet.urdf",
						basePosition=pancake_init_pos,
						baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
	p.changeVisualShape(robotId, 12, rgbaColor=[i / 255.0 for i in color_robot])
	p.resetDebugVisualizerCamera(0.2, 90, -15, [0, -0.05, 0.2])
	# end init bullet


	pan = world.loadSkeleton("./urdf/pan3/pan3.urdf", np.array([-0.1, -0.07, 0.025]), np.array([0, 0, 0]))
	pan.getBodyNodes()[0].setCollidable(False)


	pancake = world.loadSkeleton("./urdf/pancake/pancake2.urdf", pancake_init_pos, np.array([0, 0, math.pi / 2])) 

	for i in range(pancake.getNumBodyNodes()):
		pancake_node = pancake.getBodyNode(i)
		pancake_node.setFrictionCoeff(1.)

	

	ikMap = nimble.neural.IKMapping(world)
	spatula_link = franka.getBodyNode("spatula_link")
	ikMap.addSpatialBodyNode(spatula_link)
	actions_robot = torch.zeros((num_timesteps, 7), requires_grad=True)
	optimizer = torch.optim.Adam([{'params': actions_robot}], lr=0.01)
	#actions_other = torch.zeros((num_timesteps, world.getNumDofs() - 7), requires_grad=False)

	init_state = torch.Tensor(world.getState())
	print("Initial state:", init_state)
	init_hand_pos = nimble.map_to_pos(world, ikMap, init_state)
	print("Initial EE:", init_hand_pos)
	target_pos = torch.Tensor(desired_goal)
	print("Target Position:", target_pos)

	if args.vis:
		gui = nimble.NimbleGUI(world)
		gui.serve(8090)
		gui.nativeAPI().renderWorld(world)

	print(world.checkPenetration(True))
	# input()

	for k in range(100000):
		# if (k-1) % 10 == 0:
		# 	input()
		state = init_state
		states = [state.detach()]
		hand_poses = [nimble.map_to_pos(world, ikMap, state).detach()]
		for i in range(num_timesteps):
			action = torch.concat([actions_robot[i], torch.zeros(8,)], -1)
			state = nimble.timestep(world, state, action)
			
			state[7:9] = 0.009
			#print(state)
			states.append(state.detach())
			hand_poses.append(nimble.map_to_pos(world, ikMap, state).detach())
		if args.vis:
			gui.loopStates(states)
		hand_pos = nimble.map_to_pos(world, ikMap, state)
		# print(hand_pos)
		loss_grasp = ((hand_pos[3:] - target_pos)**2).mean() *10 + 0.01 * ((torch.abs(hand_pos[:3]) - torch.Tensor([0, 2.6436, 1.6974]))**2).mean()
		loss_stop = (state[15:22]**2).mean()
		# loss_stop = (state[10:16]**2).mean()
		# loss_stop = (state[9:]**2).mean()
		# print(state)
		loss = loss_grasp + 0.01*loss_stop
		print("move", k, hand_pos[5].detach().numpy(), loss.detach().numpy())
		
		
		optimizer.zero_grad()
		loss.backward()
		if np.isnan(actions_robot.grad.numpy().sum()):
			break
		optimizer.step()
		# input()
		if loss < 1e-4:
			flag = True
			break

	torch.save(states, './data/pancake_lift_thin_nimble.pt')

	# start update bullet
	def set_robot_state(state):
		joint_indices = [1, 2, 3, 4, 5, 6, 7, 10, 11]
		for idx, val in zip(joint_indices, state):
			p.resetJointState(bodyUniqueId=robotId, jointIndex=idx, targetValue=val)

	def set_pancake_state(pos, euler):
		p.resetBasePositionAndOrientation(pancakeId, np.array(pos) + pancake_init_pos, p.getQuaternionFromEuler(euler))

	set_robot_state(robot_init_state)

	for t, state in enumerate(states):
		robot_state = state[:9]
		pancake_euler, pancake_pos = state[9:12], state[12:15]
		set_robot_state(robot_state)
		set_pancake_state(pancake_pos, pancake_euler)
		# p.stepSimulation()

		time.sleep(0.1)
	# end update bullet

	if args.vis:
		gui.loopStates(states)
		gui.blockWhileServing()
		gui.stopServing()

	# input()

	# _states = np.concatenate(states).reshape(-1,18)
	# joint_actions = _states[1:,:7] - _states[:num_timesteps,:7]

	# torch.save(states, './data/peg_insertion_big_005_nimble.pt')
	# if args.vis:
	# 	gui.loopStates(states)
	# 	gui.blockWhileServing()
	# 	gui.stopServing()

	return joint_actions, flag


if __name__ == "__main__":
	start_time = time.time()
	parser = argparse.ArgumentParser()
	parser.add_argument("--env", default="PandaPancakeJointsDense-v2")  # OpenAI gym environment name
	parser.add_argument("--exp_name", default="exp")
	parser.add_argument("--discount", default=0.9)
	parser.add_argument("--lr", default=0.5)
	parser.add_argument("--seed", default=970101, type=int)
	parser.add_argument("--num_timesteps", default=30, type=int)
	parser.add_argument("--data_dir", default="./data/demo")
	parser.add_argument("--rand_cam", action="store_true")
	parser.add_argument("--camera_mode", default="8dofreal")
	parser.add_argument("--vis", action="store_true")
	args = parser.parse_args()

	num_timesteps = args.num_timesteps
	

	
	pancake_init_pos = np.array((-0.1, -0.09, 0.063))
	# pancake_init_pos = np.array((-0.1, -0.1, 0.07))
	desired_goal = np.array((-0.1, -0.1, 0.1))
	# desired_goal = np.array((-0.1, -0.1, 0.052))

	joint_actions, success = model_base_assemble(desired_goal, pancake_init_pos, num_timesteps)
	if success:
		print('success')
