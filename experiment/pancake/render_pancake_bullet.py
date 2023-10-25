import os
import math
import time
import torch
import numpy as np
import pybullet as p
import pybullet_data


def render(traj_path):
    states = torch.load(traj_path)

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

    os.makedirs("./videos", exist_ok=True)
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, f"./videos/{traj_path.split('/')[-1].split('.')[0]}.mp4")

    def set_robot_state(state):
        joint_indices = [1, 2, 3, 4, 5, 6, 7, 10, 11]
        for idx, val in zip(joint_indices, state):
            p.resetJointState(bodyUniqueId=robotId, jointIndex=idx, targetValue=val)

    def set_pancake_state(pos, euler):
        p.resetBasePositionAndOrientation(pancakeId, np.array(pos) + pancake_init_pos, p.getQuaternionFromEuler(euler))

    robot_init_state = [0.07129488, -0.82724565, -2.68480527, -1.54026022, 0.20306757, 1.29076029,
                        -0.25689644, 0.0085, 0.0085]
    set_robot_state(robot_init_state)

    for t, state in enumerate(states):
        robot_state = state[:9]
        pancake_euler, pancake_pos = state[9:12], state[12:15]
        set_robot_state(robot_state)
        set_pancake_state(pancake_pos, pancake_euler)
        # p.stepSimulation()

        time.sleep(0.1)

    p.stopStateLogging(log_id)


if __name__ == "__main__":
    render(traj_path="./data/pancake_lift_jade.pt")
    # render(traj_path="./data/pancake_lift_nimble.pt")
