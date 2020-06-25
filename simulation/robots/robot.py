import pybullet
import numpy as np
import math
#from robots import NeuralNet
from gym import spaces

# BASE CLASS FOR ROBOTS

# Uses a lot of stuff from pybulletgym


class Robot:

  def __init__(self):
    """ Base Robot, inherit from me!
		"""
    self.parts = None
    self.objects = []
    self.jdict = None
    self.ordered_joints = None
    self.robot_body = None

    self.observation_size = 0
    self.action_size = 0

    self.observation_space = spaces.Box(-1., 1., shape=(self.observation_size,))
    self.action_size = spaces.Box(-1., 1., shape=(self.observation_size,))

    self.model_urdf = None  # Path to file
    self.basePosition = [0, 0, 0]
    self.baseOrientation = [0, 0, 0, 1]

    self.loaded = False

  def load(self):
    bodies = self._p.loadURDF(
        self.model_urdf,
        basePosition=self.basePosition,
        baseOrientation=self.baseOrientation,
        useFixedBase=True)

    if np.isscalar(bodies):
      bodies = [bodies]

    self.parts = {}
    self.joints = {}

    for bodyID in bodies:
      self.parts['torso'] = BodyPart(self._p, bodyID, -1)

      if self._p.getNumJoints(bodyID) == 0:
        part_name, robot_name = self._p.getBodyInfo(bodyID)
        self.robot_name.decode("utf8")
        part_name = part_name.decode("utf8")
      for j in range(self._p.getNumJoints(bodyID)):
        #self._p.setJointMotorControl2(bodyID, j, pybullet.POSITION_CONTROL, positionGain=0.1, velocityGain=0.1, force=0)
        jointInfo = self._p.getJointInfo(bodyID, j)
        joint_name = jointInfo[1].decode("utf8")
        part_name = jointInfo[12].decode("utf8")

        self.parts[part_name] = BodyPart(self._p, bodyID, j)
        self.joints[joint_name] = Joint(self._p, bodyID, j)

        self.joints[joint_name].disable_motor()

  def compute_observation(self):
    raise NotImplementedException

  def set_motors(self, action):
    pass

  def failed(self):
    pass

  def reward(self):
    return 0

  def robot_specific_reset(self, physicsClient):
    pass

  def reset_pose(self, position, orientation):
    self.parts[self.robot_name].reset_pose(position, orientation)

  def reset(self, bullet_client):
    self._p = bullet_client
    self.ordered_joints = []

    full_path = self.model_urdf

    if not self.loaded:
      self.load()
      self.loaded = True

    self.robot_specific_reset(self._p)

    s = self.compute_observation()
    return s


class HardcodedProgram:

  def __init__(self):
    pass

  def forward(self, observation):
    pass


def clamp(n, minn, maxn):
  return max(min(maxn, n), minn)


class BodyPart:

  def __init__(self, bullet_client, bodyID, partID):
    self.bodyID = bodyID
    self.partID = partID
    self._p = bullet_client

    self.initialPosition = self.get_position()
    self.initialOrientation = self.get_orientation()

  def state_fields_of_pose_of(
      self,
      body_id,
      link_id=-1):  # a method you will most probably need a lot to get pose and orientation
    if link_id == -1:
      (x, y, z), (a, b, c, d) = self._p.getBasePositionAndOrientation(body_id)
    else:
      (x, y, z), (a, b, c, d), _, _, _, _ = self._p.getLinkState(body_id, link_id)
    return np.array([x, y, z, a, b, c, d])

  def get_pose(self):
    return self.state_fields_of_pose_of(self.bodyID, self.partID)

  def get_velocity(self):
    if self.partID == -1:
      (vx, vy, vz), (wx, wy, wz) = self._p.getBaseVelocity(self.bodyID)
    else:
      (x, y, z), (a, b, c, d), _, _, _, _, (vx, vy, vz), (wx, wy, wz) = self._p.getLinkState(
          self.bodyID, self.partID, computeLinkVelocity=1)
    return np.array([vx, vy, vz]), np.array(wx, wy, wz)

  def get_linear_velocity(self):
    return self.get_velocity()[0]

  def get_angular_velocity(self):
    return self.get_angular_velocity()[0]

  def get_position(self):
    return self.get_pose()[:3]

  def get_orientation(self):
    return self.get_pose()[3:]

  def get_euler_orientation(self):
    return self._p.getEulerFromQuaternion(self.get_pose()[3:])

  def get_velocity(self):
    return self._p.getBaseVelocity(self.bodyID)

  def reset_position(self, position):
    self._p.resetBasePositionAndOrientation(self.bodyID, position, self.get_orientation())

  def reset_orientation(self, orientation):
    self._p.resetBasePositionAndOrientation(self.bodyID, self.get_position(), orientation)

  def reset_velocity(self, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0]):
    self._p.resetBaseVelocity(self.bodyID, linearVelocity, angularVelocity)

  def contact_list(self):
    return self._p.getContactPoints(self.bodies[self.bodyIndex], -1, self.bodyPartIndex, -1)


class Joint:
  JOINT_REVOLUTE_TYPE = 0
  JOINT_PLANAR_TYPE = 1
  JOINT_PRISMATIC_TYPE = 2
  JOINT_SPHERICAL_TYPE = 3
  JOINT_FIXED_TYPE = 4

  def __init__(self, bullet_client, bodyID, jointID):
    self._p = bullet_client
    self.bodyID = bodyID
    self.jointID = jointID

    joint_info = self._p.getJointInfo(self.bodyID, self.jointID)
    self.jointType = joint_info[2]
    self.lowerLimit = joint_info[8]
    self.upperLimit = joint_info[9]
    self.jointHasLimits = self.lowerLimit < self.upperLimit
    self.jointMaxVelocity = joint_info[11]
    self.power_coeff = 0

  def set_state(self, x, vx):
    self._p.resetJointState(self.bodyID, self.jointID, x, vx)

  """
	def get_relative_position(self):
		pos, vel = self.get_state()
		if self.jointHasLimits:
			pos_mid = 0.5 * (self.lowerLimit + self.upperLimit)
			pos = 2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit)

		if self.jointMaxVelocity > 0:
			vel /= self.jointMaxVelocity
		elif self.jointType == 0:  # JOINT_REVOLUTE_TYPE
			vel *= 0.1
		else:
			vel *= 0.5
		return (
			pos,
			vel
		)
	"""

  def get_state(self):
    x, vx, _, _ = self._p.getJointState(self.bodyID, self.jointID)
    return x, vx

  def get_position(self):
    x, _ = self.get_state()
    return x

  def get_velocity(self):
    _, vx = self.get_state()
    return vx

  def set_position(self, position):
    self._p.setJointMotorControl2(
        self.bodyID, self.jointID, pybullet.POSITION_CONTROL, targetPosition=position)

  def set_velocity(self, velocity):
    self._p.setJointMotorControl2(
        self.bodyID, self.jointID, pybullet.VELOCITY_CONTROL, targetVelocity=velocity)

  def set_torque(self, torque):
    self._p.setJointMotorControl2(
        bodyIndex=self.bodyID,
        jointIndex=self.jointID,
        controlMode=pybullet.TORQUE_CONTROL,
        force=torque)  # positionGain=0.1, velocityGain=0.1)

  def reset_position(self, position, velocity=0):
    self._p.resetJointState(
        self.bodyID, self.jointID, targetValue=position, targetVelocity=velocity)
    self.disable_motor()

  def disable_motor(self):
    self._p.setJointMotorControl2(
        self.bodyID,
        self.jointID,
        controlMode=pybullet.VELOCITY_CONTROL,
        targetVelocity=0,
        positionGain=0.1,
        velocityGain=0.1,
        force=0.1)
