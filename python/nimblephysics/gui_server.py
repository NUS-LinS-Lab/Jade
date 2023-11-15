from http.server import HTTPServer, SimpleHTTPRequestHandler, ThreadingHTTPServer
from http import HTTPStatus
import os
import pathlib
import nimblephysics as nimble
import random
import typing
import threading
from typing import List
import torch
import numpy as np
import math
import pybullet as p
import pybullet_data
import time

file_path = os.path.join(pathlib.Path(__file__).parent.absolute(), 'web_gui')


def createRequestHandler():
  """
  This creates a request handler that can serve the raw web GUI files, in
  addition to a configuration string of JSON.
  """
  class LocalHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
      super().__init__(*args, directory=file_path, **kwargs)

    def do_GET(self):
      """
      if self.path == '/json':
          resp = jsonConfig.encode("utf-8")
          self.send_response(HTTPStatus.OK)
          self.send_header("Content-type", "application/json")
          self.send_header("Content-Length", len(resp))
          self.end_headers()
          self.wfile.write(resp)
      else:
          super().do_GET()
      """
      super().do_GET()
  return LocalHTTPRequestHandler


class NimbleGUI:
  def __init__(self, worldToCopy: nimble.simulation.World, useBullet=False, video_log_file=None):
    self.useBullet = useBullet
    if useBullet:
      self.render_bullet_init(worldToCopy)
      self.log_id = None
      self.world = worldToCopy
      if video_log_file is not None:
        video_log_dir = os.path.dirname(video_log_file)
        os.makedirs(video_log_dir, exist_ok=True)
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_log_file)
    else:
      self.world = worldToCopy.clone()
      self.guiServer = nimble.server.GUIWebsocketServer()
      self.guiServer.renderWorld(self.world)
      # Set up the realtime animation
      self.ticker = nimble.realtime.Ticker(self.world.getTimeStep() * 10)
      self.ticker.registerTickListener(self._onTick)
      self.guiServer.registerConnectionListener(self._onConnect)

      self.looping = False
      self.posMatrixToLoop = np.zeros((self.world.getNumDofs(), 0))
      self.i = 0

  def serve(self, port):
    if self.useBullet:
      print("No need to call this function for bullet")
      return
    self.guiServer.serve(8070)
    server_address = ('', port)
    self.httpd = ThreadingHTTPServer(server_address, createRequestHandler())
    print('Web GUI serving on http://localhost:'+str(port))
    t = threading.Thread(None, self.httpd.serve_forever)
    t.daemon = True
    t.start()

  def stopServing(self):
    if self.useBullet:
      if self.log_id is not None:
        p.stopStateLogging(self.log_id)
      p.disconnect()
      return
    self.guiServer.stopServing()
    self.httpd.shutdown()

  def displayState(self, state: torch.Tensor):
    self.looping = False
    self.world.setState(state.detach().numpy())
    self.guiServer.renderWorld(self.world)

  def loopStates(self, states: List[torch.Tensor]):
    if self.useBullet:
      for state in states:
        self.bullet_loopState(state)
        time.sleep(0.1)
    self.looping = True
    self.statesToLoop = states
    dofs = self.world.getNumDofs()
    poses = np.zeros((dofs, len(states)))
    for i in range(len(states)):
      # Take the top-half of each state vector, since this is the position component
      poses[:, i] = states[i].detach().numpy()[:dofs]
    self.guiServer.renderTrajectoryLines(self.world, poses)
    self.posMatrixToLoop = poses

  def loopPosMatrix(self, poses: np.ndarray):
    self.looping = True
    self.guiServer.renderTrajectoryLines(self.world, poses)
    # It's important to make a copy, because otherwise we get a reference to internal C++ memory that gets cleared
    self.posMatrixToLoop = np.copy(poses)

  def stopLooping(self):
    self.looping = False

  def nativeAPI(self) -> nimble.server.GUIWebsocketServer:
    return self.guiServer

  def blockWhileServing(self):
    self.guiServer.blockWhileServing()

  def _onTick(self, now):
    if self.looping:
      if self.i < np.shape(self.posMatrixToLoop)[1]:
        self.world.setPositions(self.posMatrixToLoop[:, self.i])
        self.guiServer.renderWorld(self.world)
        self.i += 1
      else:
        self.i = 0

  def test(self):
    print("world dir:\n", dir(self.world))

    print("skeleton dir:\n", dir(self.world.getSkeleton(0)))

    for i in range(self.world.getNumSkeletons()):
      skel = self.world.getSkeleton(i)
      print("i = {}".format(i))
      print(skel)
      print("name = {}".format(skel.getName()))
      # print("pos = {}".format(self.world.getPositions()))   # robot init state
      print("pos = {}".format(skel.getPositions()))           # 似乎没有存name和pos和angle，可以在dart/utils/UniversalLoader.cpp那里存，然后再读取
      print("pos1 = {}".format(skel.getBasePos()))
      skel.setBasePos(np.array([-0.39, 0.075, 0.0]))
      print("pos2 = {}".format(skel.getBasePos()))
      print("angle1 = {}".format(skel.getEulerAngle()))
      skel.setEulerAngle(np.array([0, 0, math.pi / 2]))
      print("angle2 = {}".format(skel.getEulerAngle()))
      print("urdf1 = {}".format(skel.getURDFPath()))
      skel.setURDFPath("./urdf/pancake/pancake2.urdf")
      print("urdf2 = {}".format(skel.getURDFPath()))

  def render_bullet_init(self, world):
    self.physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setTimeStep(0.01)
    p.setGravity(0, 0, 0)

    self.skeleton_to_bullet_id = {}
    self.init_pos_rot = {}

    for i in range(world.getNumSkeletons()):
      skeleton = world.getSkeleton(i)
      urdf_path = skeleton.getURDFPath()
      pos = skeleton.getRootBodyNode().getTransform().translation()
      rot = skeleton.getRootBodyNode().getTransform().rotation()
      bullet_id = p.loadURDF(urdf_path, pos, rot)
      self.skeleton_to_bullet_id[skeleton.getName()] = bullet_id
      self.init_pos_rot[skeleton.getName()] = (pos, rot)

  def bullet_loopState(self, state):
    tick = 0
    for skeleton_idx in range(self.world.getNumSkeletons()):
      skeleton = self.world.getSkeleton(skeleton_idx)
      dof = skeleton.getNumDofs()
      if dof == 0:
        continue
      p_id = self.skeleton_to_bullet_id[skeleton.getName()]
      actions = state[tick: tick+dof]
      joint_infos = [p.getJointInfo(p_id,i)[2] for i in range(p.getNumJoints(p_id))]
      non_fixed_joint_ids = [joint_id for joint_id, joint_info in enumerate(joint_infos) if joint_info != p.JOINT_FIXED]
      if len(non_fixed_joint_ids) == 0:
        init_pos, init_angle = self.init_pos_rot[skeleton.getName()]
        p.resetBasePositionAndOrientation(p_id, np.array(actions[:3]) + init_pos,
                                          p.getQuaternionFromEuler(init_angle + np.array(actions[3:])))
      else:
        for i, joint_id in enumerate(non_fixed_joint_ids):
          p.resetJointState(p_id, joint_id, actions[i])
      tick += dof

  def _onConnect(self):
    self.ticker.start()