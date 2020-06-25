import numpy as np
import pybullet as p


class Sensor:
    """ Sensor that allows specifying some random noise and error
    """

    def __init__(self, noise_std=0, offset_std=0):
        self.noise_std = noise_std
        self.offset = np.random.randn() * offset_std

    def query(self):
        pass


class Accelerometer(Sensor):
    """ Accelerometer that returns pitch, yaw, roll that allows specifying some random noise and error
    """

    def __init__(self, bodyPart, sensorId=-1, noise_std=0, offset_std=0):
        super().__init__(noise_std, offset_std)
        self.bodyPart = bodyPart

    def query(self):
        eulerOrientation = self.bodyPart.get_euler_orientation()
        return [axisOrientation + self.offset + self.noise_std * np.random.randn() for axisOrientation in eulerOrientation]


class Encoder(Sensor):
    """ Encoder that allows specifying some random noise and error
    """

    def __init__(self, joint, noise_std=0, offset_std=0):
        super().__init__(noise_std, offset_std)
        self.joint = joint

    def query(self):
        return self.joint.get_position() + self.offset + self.noise_std * np.random.randn()


class Gyro(Sensor):
    """ Gyro that reeturns pitch rate, yaw rate, roll rate and allows specifying some random noise and error
    """

    def __init__(self, bodyPart, noise_std=0, offset_std=0):
        super().__init__(noise_std, offset_std)
        self.bodyPart = bodyPart

    def query(self):
        linearVelocity, angularVelocity = self.bodyPart.get_velocity()

        linearVelocity = [axisOrientation + self.offset + self.noise_std * np.random.randn() for axisOrientation in linearVelocity]
        angularVelocity = [axisOrientation + self.offset + self.noise_std * np.random.randn() for axisOrientation in angularVelocity]
        return linearVelocity, angularVelocity
