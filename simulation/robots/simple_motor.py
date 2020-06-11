import numpy as np
from robots import clamp
import pybullet as p

class Motor:
    """ Sensor that allows specifying some random noise and error
    """
    def __init__(self, joint, maxTorque = 2.46,  noise_std = 0, offset_std = 0):
        self.joint = joint

        self._p = self.joint._p
        self.botId = self.joint.bodyID
        self.motorId = self.joint.jointID

        self.noise_std = noise_std
        self.offset = np.random.randn()*offset_std

        self.maxTorque = maxTorque

        self._p.setJointMotorControl2(bodyUniqueId=self.botId,
                                      jointIndex=self.motorId,
                                      controlMode=p.VELOCITY_CONTROL,
                                      force=0.5)

    def setMotorVelocity(self, counts_per_sec):
        raise NotImplementedError
        #speed = clamp(speed, -1,1)*self.maxSpeed
        #p.setJointMotorControl2(self.botId, self.motorId, p.VELOCITY_CONTROL, force=0.5)
        self._p.setJointMotorControl2(bodyUniqueId=self.botId,
                                jointIndex=self.motorId,
                                targetVelocity=counts_per_sec,
                                controlMode=p.VELOCITY_CONTROL,
                                force = self.maxTorque)

    def setMotorPosition(self, angle_radians):
        proportional = angle_radians - self.joint.get_position()
        derivative = 0 - self.joint.get_velocity()

        k_p = 7
        k_d = 0
        torque = k_p * proportional + k_d * derivative

        torque = max(min(self.maxTorque*torque, self.maxTorque), -self.maxTorque)
        """
        self._p.setJointMotorControl2(bodyUniqueId=self.botId,
                                      jointIndex=self.motorId,
                                      controlMode=p.TORQUE_CONTROL,
                                      force=torque)
        """
        self._p.setJointMotorControl2(bodyUniqueId=self.botId,
                                jointIndex=self.motorId,
                                targetPosition=angle_radians,
                                controlMode=p.POSITION_CONTROL)
