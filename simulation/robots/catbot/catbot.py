import pybullet as p
import numpy as np
import math

from robots import *
from gym import spaces


class catbot(Robot):

  def __init__(self):
    super().__init__()
    self.model_urdf = 'robots/catbot/urdf/catbot.urdf'

    self.observation_space = spaces.Box(-1., 1., shape=(6,))
    self.action_space = spaces.Box(-1., 1., shape=(12,))

    self.energy = 0

    self.basePosition = [0, 0, 0.45]
    self.baseOrientation = [0, 0, 0, 1]

  def robot_specific_reset(self, bullet_client):
    self._p = bullet_client
    for j in self.joints.values():
      j.reset_position(self.np_random.uniform(low=-0.1, high=0.1), 0)

    self.accelerometer = sensors.Accelerometer(self.parts['torso'], noise_std=0.1, offset_std=0.1)
    self.gyro = sensors.Gyro(self.parts['torso'], noise_std=0.1, offset_std=0.1)

    self.motor_torque = 20

    self.LF_0 = Motor(self.joints['LF_0'], maxTorque=self.motor_torque)
    self.LF_1 = Motor(self.joints['LF_1'], maxTorque=self.motor_torque)
    self.LF_2 = Motor(self.joints['LF_2'], maxTorque=self.motor_torque)

    self.RF_0 = Motor(self.joints['RF_0'], maxTorque=self.motor_torque)
    self.RF_1 = Motor(self.joints['RF_1'], maxTorque=self.motor_torque)
    self.RF_2 = Motor(self.joints['RF_2'], maxTorque=self.motor_torque)

    self.LB_0 = Motor(self.joints['LB_0'], maxTorque=self.motor_torque)
    self.LB_1 = Motor(self.joints['LB_1'], maxTorque=self.motor_torque)
    self.LB_2 = Motor(self.joints['LB_2'], maxTorque=self.motor_torque)

    self.RB_0 = Motor(self.joints['RB_0'], maxTorque=self.motor_torque)
    self.RB_1 = Motor(self.joints['RB_1'], maxTorque=self.motor_torque)
    self.RB_2 = Motor(self.joints['RB_2'], maxTorque=self.motor_torque)

  def set_motors(self, normalized_action):
    #print(normalized_action)
    action = normalized_action
    #action = [math.pi * a for a in normalized_action]
    #print(action)

    #action = [0 for a in normalized_action]
    self.energy = sum(normalized_action)

    LF_0, LF_1, LF_2, RF_0, RF_1, RF_2, LB_0, LB_1, LB_2, RB_0, RB_1, RB_2 = action

    #LF_0 = RF_0 = LB_0 = RB_0 = 0
    #LF_1 = -RF_1
    #LF_2 = -RF_2
    #LB_1 = -RB_1
    #LB_2 = -RB_2

    self.LF_0.setMotorPosition(LF_0)
    self.LF_1.setMotorPosition(LF_1)
    self.LF_2.setMotorPosition(LF_2)

    self.RF_0.setMotorPosition(RF_0)
    self.RF_1.setMotorPosition(RF_1)
    self.RF_2.setMotorPosition(RF_2)

    self.LB_0.setMotorPosition(LB_0)
    self.LB_1.setMotorPosition(LB_1)
    self.LB_2.setMotorPosition(LB_2)

    self.RB_0.setMotorPosition(RB_0)
    self.RB_1.setMotorPosition(RB_1)
    self.RB_2.setMotorPosition(RB_2)

  def compute_observation(self):

    pitch, yaw, roll = self.accelerometer.query()
    linearVelocity, (pitch_rate, roll_rate, yaw_rate) = self.gyro.query()

    observation = np.array([pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate])
    return observation

  def failed(self):
    torsoPosition = self.parts['torso'].get_position()
    torsoOrn = self.accelerometer.query()

    if (torsoPosition[2] < 0.2 or max(abs(n) for n in torsoOrn) > 0.15 * math.pi or self.parts['LF_bot'].get_position()[2] < 0.02 or self.parts['RF_bot'].get_position()[2] < 0.02 or self.parts['LB_bot'].get_position()[2] < 0.02 or self.parts['RB_bot'].get_position()[2] < 0.02):
      return True

    return False

  def reward(self):
    linearVelocity, angularVelocity = self.gyro.query()
    return -linearVelocity[1]


def clamp(n, minn, maxn):
  return max(min(maxn, n), minn)
