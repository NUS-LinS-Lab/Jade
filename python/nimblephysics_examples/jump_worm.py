import numpy as np
import torch
import torch.nn.functional as F
import random
import math
import time
import diffdart as dart
from typing import Dict
from diffdart import DartTorchLossFn, DartTorchTrajectoryRollout, GUITrajectoryTrainer


def main():
  world = dart.simulation.World()
  world.setGravity([0, -9.81, 0])

  # Set up the 2D cartpole

  jumpworm = dart.dynamics.Skeleton()

  rootJoint, root = jumpworm.createTranslationalJoint2DAndBodyNodePair()
  rootJoint.setXYPlane()
  rootShape = root.createShapeNode(dart.dynamics.BoxShape([.1, .1, .1]))
  rootVisual = rootShape.createVisualAspect()
  rootShape.createCollisionAspect()
  rootVisual.setColor([0.7, 0.7, 0.7])
  rootJoint.setControlForceUpperLimit(0, 0)
  rootJoint.setControlForceLowerLimit(0, 0)
  rootJoint.setControlForceUpperLimit(1, 0)
  rootJoint.setControlForceLowerLimit(1, 0)
  rootJoint.setVelocityUpperLimit(0, 1000.0)
  rootJoint.setVelocityLowerLimit(0, -1000.0)
  rootJoint.setVelocityUpperLimit(1, 1000.0)
  rootJoint.setVelocityLowerLimit(1, -1000.0)

  def createTailSegment(parent, color):
    poleJoint, pole = jumpworm.createRevoluteJointAndBodyNodePair(parent)
    poleJoint.setAxis([0, 0, 1])
    poleShape = pole.createShapeNode(dart.dynamics.BoxShape([.05, 0.25, .05]))
    poleVisual = poleShape.createVisualAspect()
    poleVisual.setColor(color)
    poleJoint.setControlForceUpperLimit(0, 100.0)
    poleJoint.setControlForceLowerLimit(0, -100.0)
    poleJoint.setVelocityUpperLimit(0, 10000.0)
    poleJoint.setVelocityLowerLimit(0, -10000.0)

    poleOffset = dart.math.Isometry3()
    poleOffset.set_translation([0, -0.125, 0])
    poleJoint.setTransformFromChildBodyNode(poleOffset)

    poleJoint.setPosition(0, 90 * 3.1415 / 180)
    poleJoint.setPositionUpperLimit(0, 180 * 3.1415 / 180)
    poleJoint.setPositionLowerLimit(0, 0 * 3.1415 / 180)

    poleShape.createCollisionAspect()

    if parent != root:
      childOffset = dart.math.Isometry3()
      childOffset.set_translation([0, 0.125, 0])
      poleJoint.setTransformFromParentBodyNode(childOffset)
    return pole

  tail1 = createTailSegment(root, [182.0/255, 223.0/255, 144.0/255])
  tail2 = createTailSegment(tail1, [223.0/255, 228.0/255, 163.0/255])
  tail3 = createTailSegment(tail2, [221.0/255, 193.0/255, 121.0/255])
  # tail4 = createTailSegment(tail3, [226.0/255, 137.0/255, 79.0/255])

  jumpworm.setPositions(np.array([0, 0, 90, 90, 45]) * 3.1415 / 180)

  world.addSkeleton(jumpworm)

  # Floor

  floor = dart.dynamics.Skeleton()
  floor.setName('floor')  # important for rendering shadows

  floorJoint, floorBody = floor.createWeldJointAndBodyNodePair()
  floorOffset = dart.math.Isometry3()
  floorOffset.set_translation([0, -0.7, 0])
  floorJoint.setTransformFromParentBodyNode(floorOffset)
  floorShape = floorBody.createShapeNode(dart.dynamics.BoxShape([2.5, 0.25, .5]))
  floorVisual = floorShape.createVisualAspect()
  floorVisual.setColor([0.5, 0.5, 0.5])
  floorShape.createCollisionAspect()
  # floorBody.setFrictionCoeff(0)

  world.addSkeleton(floor)

  # Set up the view

  def loss(rollout: DartTorchTrajectoryRollout):
    pos = rollout.getPoses('ik')
    vel = rollout.getVels('ik')
    step_loss = - torch.sum(pos[1, :] * pos[1, :] * torch.sign(pos[1, :]))
    last_pos_y = pos[1, -1]
    last_vel_y = vel[1, -1]
    final_loss = - 100 * torch.square(last_pos_y) * torch.sign(last_pos_y)
    return final_loss
  dartLoss: dart.trajectory.LossFn = DartTorchLossFn(loss)

  trajectory = dart.trajectory.MultiShot(world, dartLoss, 400, 20, False)

  ikMap: dart.neural.IKMapping = dart.neural.IKMapping(world)
  ikMap.addLinearBodyNode(root)
  trajectory.addMapping('ik', ikMap)

  trajectory.setParallelOperationsEnabled(True)
  optimizer = dart.trajectory.IPOptOptimizer()
  optimizer.setLBFGSHistoryLength(3)
  optimizer.setTolerance(1e-5)
  optimizer.setCheckDerivatives(False)
  optimizer.setIterationLimit(500)
  optimizer.setRecordPerformanceLog(True)

  trainer = GUITrajectoryTrainer(world, trajectory, optimizer)
  result: dart.trajectory.Solution = trainer.train(True)
  # perflogs: Dict[str, dart.performance.FinalizedPerformanceLog] = result.getPerfLog().finalize()
  # print(perflog)

  """
  json = result.toJson(world)
  text_file = open("worm.txt", "w")
  n = text_file.write(json)
  text_file.close()

  dart.dart_serve_optimization_solution(result, world)
  """


if __name__ == "__main__":
  main()
