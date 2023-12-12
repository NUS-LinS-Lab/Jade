from http.server import HTTPServer, SimpleHTTPRequestHandler, ThreadingHTTPServer
from http import HTTPStatus
import os
import pathlib
import nimblephysics as nimble
import random
import typing
import threading
from typing import Any, List
import torch
import numpy as np
import math
import pybullet as p
import pybullet_data
import time
# from scipy.spatial.transform import Rotation
import pdb

file_path = os.path.join(pathlib.Path(__file__).parent.absolute(), 'web_gui')

class DeprecatedClass:
  def __getattribute__(self, __name: str) -> Any:
    def deprecated_func(*args, **kwargs):
      print(f"WARNING: No need to call <{__name}> in bullet vis mode")
    
    if __name.startswith('__') and __name.endswith('__'):
      return object.__getattribute__(self,__name)
    else:
      return deprecated_func
    
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
  def __init__(self, worldToCopy: nimble.simulation.World, 
               useBullet=False,
               useSyntheticCamera=True,
               saveCameraPath=None,
               videoLogFile=None):
    self.useBullet = useBullet
    if useBullet:
      self.log_id = None
      self.saveCameraPath = None
      self.useSyntheticCamera = useSyntheticCamera
      self.render_bullet_init(worldToCopy)
      if videoLogFile is not None:
        video_log_dir = os.path.dirname(videoLogFile)
        os.makedirs(video_log_dir, exist_ok=True)
        self.log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, videoLogFile)
      if saveCameraPath is not None:
        self.saveCameraPath = os.path.abspath(saveCameraPath)
        os.makedirs(self.saveCameraPath, exist_ok=True)
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

  def loopStates(self, states: List[torch.Tensor],
                 indefinite: bool=False,
                 save_start_idx: int=0):
    if self.useBullet:
      while True:
        for i, state in enumerate(states):
          self.bullet_loopState(state, i+save_start_idx)
          time.sleep(0.1)
        if not indefinite:
          break
      return
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
    if self.useBullet:
      print("No need to call this function for bullet")
      return DeprecatedClass()
    return self.guiServer

  def blockWhileServing(self):
    if self.useBullet:
      return
    self.guiServer.blockWhileServing()

  def _onTick(self, now):
    if self.looping:
      if self.i < np.shape(self.posMatrixToLoop)[1]:
        self.world.setPositions(self.posMatrixToLoop[:, self.i])
        self.guiServer.renderWorld(self.world)
        self.i += 1
      else:
        self.i = 0

  def bullet_reset(self, world):
    p.resetSimulation()
    self.world = world
    self.skeleton_to_bullet_id = {}
    self.init_pos_rot = {}

    for i in range(world.getNumSkeletons()):
      skeleton = world.getSkeleton(i)
      urdf_path = skeleton.getURDFPath()
      # pdb.set_trace()
      # pos = skeleton.getRootBodyNode().getTransform().translation()
      # pdb.set_trace()
      # rot_quat = skeleton.getRootBodyNode().getTransform().quaternion().wxyz()
      # rot = Rotation.from_quat(np.concatenate((rot_quat[1:], rot_quat[:1])))
      # rot = _rot
      # rot = Rotation.from_euler('zyx', _rot.as_euler('xyz'), degrees=False)
      pos = skeleton.getBasePos()
      rot = skeleton.getEulerAngle()
      rot_quat = p.getQuaternionFromEuler(rot)

      # print("urdf_path = {}".format(urdf_path))
      # print(type(pos), type(rot))
      # print(rot_mat)
      # print(pos, rot, rot_quat)
      bullet_id = p.loadURDF(urdf_path, pos, rot_quat)
      # p.resetBasePositionAndOrientation(bullet_id, pos, rot_quat)
      self.skeleton_to_bullet_id[skeleton.getName()] = bullet_id
      self.init_pos_rot[skeleton.getName()] = (pos, rot) 

  def render_bullet_init(self, world):
    self.p = p
    self.gui_id = p.connect(p.GUI)
    if self.useSyntheticCamera:
      p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
      p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
      p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
    else:
      p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
      
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setTimeStep(0.01)
    p.setGravity(0, 0, 0)

    self.bullet_reset(world)
    self.bullet_loopState(world.getState(), None)             
    self.bullet_auto_camera()
    
  def bullet_loopState(self, state, save_idx):
    tick = 0
    for skeleton_idx in range(self.world.getNumSkeletons()):
      skeleton = self.world.getSkeleton(skeleton_idx)
      dof = skeleton.getNumDofs()
      if dof == 0:
        continue
      
      p_id = self.skeleton_to_bullet_id[skeleton.getName()]
      actions = state[tick: tick+dof]
      # print(p_id, skeleton.getName(), actions)
      
      joint_infos = [p.getJointInfo(p_id,i)[2] for i in range(p.getNumJoints(p_id))]
      non_fixed_joint_ids = [joint_id for joint_id, joint_info in enumerate(joint_infos) if joint_info != p.JOINT_FIXED]
      if len(non_fixed_joint_ids) == 0:
        init_pos, init_angle = self.init_pos_rot[skeleton.getName()]
        pos_change, angle_change = np.array(actions[3:]), np.array(actions[:3])
        # print(f'{p_id}: name = {skeleton.getName()}, pos = {pos_change + init_pos}, angle = {init_angle + angle_change}')
        p.resetBasePositionAndOrientation(p_id, pos_change + init_pos,
                                          p.getQuaternionFromEuler(init_angle + angle_change))
      else:
        for i, joint_id in enumerate(non_fixed_joint_ids):
          p.resetJointState(p_id, joint_id, actions[i])
      tick += dof
    
    if self.useSyntheticCamera:
      img = p.getCameraImage(640, 480, renderer=p.ER_BULLET_HARDWARE_OPENGL)
      if (self.saveCameraPath is not None) and (save_idx is not None):
        rgb = img[2]
        depth = img[3]
        segmentation = img[4]
        rgb = np.reshape(rgb, (480, 640, 4))
        depth = np.reshape(depth, (480, 640))
        segmentation = np.reshape(segmentation, (480, 640))
        np.save(os.path.join(self.saveCameraPath, f'rgb_{save_idx}.npy'), rgb)
        np.save(os.path.join(self.saveCameraPath, f'depth_{save_idx}.npy'), depth)
        np.save(os.path.join(self.saveCameraPath, f'segmentation_{save_idx}.npy'), segmentation)

  def bullet_auto_camera(self):
    inf = float('inf')
    aabb_mins, aabb_maxs = [inf, inf, inf], [-inf, -inf, -inf]
    for p_id in self.skeleton_to_bullet_id.values():
      aabb = p.getAABB(p_id)
      # print(aabb)
      aabb_mins = [min(aabb_mins[i], aabb[0][i]) for i in range(3)]
      aabb_maxs = [max(aabb_maxs[i], aabb[1][i]) for i in range(3)]
    center = [(aabb_mins[i] + aabb_maxs[i]) / 2 for i in range(3)]
    diagonal = sum([(aabb_maxs[i] - aabb_mins[i]) ** 2 for i in range(3)]) ** 0.5
    camera_dist = diagonal * 2
    camera_yaw, camera_pitch = 40, -20
    # print("camera_dist = {}, camera_yaw = {}, camera_pitch = {}".format(camera_dist, camera_yaw, camera_pitch))
    p.resetDebugVisualizerCamera(cameraDistance=camera_dist, 
                                 cameraYaw=camera_yaw, 
                                 cameraPitch=camera_pitch,
                                 cameraTargetPosition=center)

  def _onConnect(self):
    self.ticker.start()