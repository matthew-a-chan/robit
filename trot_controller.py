import numpy as np
import ik
#from rl_algorithms.CMA import torchnet


class LegController:

    def __init__(self, mode='forward'):
        self.target_velocity = np.array([0., 1., 0.])  # right, forward, up

        self.forward_cycle_length = 50
        self.reverse_cycle_length = 50
        self.timestep = 0
        self.mode = mode

    def get_position(self):
        self.forward_point = self.target_velocity / (np.linalg.norm(self.target_velocity) + 1e-5) * 100
        self.reverse_point = -self.forward_point

        if self.mode == 'forward':
            target_position = lerp(self.reverse_point, self.forward_point, self.timestep / self.forward_cycle_length)
            target_position[2] += np.sin((self.timestep / self.forward_cycle_length) * 3.14159) * 70
        elif self.mode == 'reverse':
            target_position = lerp(self.forward_point, self.reverse_point, self.timestep / self.reverse_cycle_length)

        self.timestep += 1
        if self.mode == 'forward' and self.timestep >= self.forward_cycle_length:
            self.timestep = 0
            self.mode = 'reverse'
        elif self.mode == 'reverse' and self.timestep >= self.reverse_cycle_length:
            self.timestep = 0
            self.mode = 'forward'

        return target_position


class TrotController:

    def __init__(self):
        self.target_velocity = np.array([0., 1., 0.])

        self.lc_FL = LegController('forward')
        self.lc_FR = LegController('reverse')
        self.lc_BL = LegController('reverse')
        self.lc_BR = LegController('forward')
        self.moving_average_roll_rate = 0
        self.moving_average_pitch_rate = 0
        self.moving_average_roll = 0
        self.moving_average_pitch = 0

        # PARAMETERS:
        #self.net = torchnet.NeuralNet([6*3, 24, 3])
        self.observations = [0., 0., 0., 0., 0., 0.] * 3

    def adopt_parameters(self, parameters):
        self.net.adopt_parameters(parameters)

    def parameter_size(self):
        return self.net.parameter_size()

    def get_positions(self, observation):
        self.observations = self.observations[6:] + list(observation)

        #self.target_velocity = self.net.predict(self.observations)[0].numpy() + np.array([0., 1., 0.])
        self.target_velocity = np.array([0, 1.0, 0])

        self.lc_FL.target_velocity = self.target_velocity
        self.lc_FR.target_velocity = self.target_velocity
        self.lc_BL.target_velocity = self.target_velocity
        self.lc_BR.target_velocity = self.target_velocity
        return self.lc_FL.get_position(), self.lc_FR.get_position(), self.lc_BL.get_position(), self.lc_BR.get_position()

    def predict(self, observation):
        corrections = np.array([-0.1, -.85, -1.55, 0.1, +.85, +1.55, 0.1, -.85, -1.55, -0.1, +.85, +1.55])
        center = np.array([0, 0, -200])
        actions = np.zeros(12)

        offset_FL, offset_FR, offset_BL, offset_BR = self.get_positions(observation)
        actions[0:3] = ik.solve(center + offset_BR, 'Back_Right')
        actions[3:6] = ik.solve(center + offset_BL, 'Back_Left')
        actions[6:9] = ik.solve(center + offset_FR, 'Front_Right')
        actions[9:12] = ik.solve(center + offset_FL, 'Front_Left')

        return corrections + actions, None  #lerp(corrections, actions + corrections, np.clip(timestep/500, 0.0, 1.0))


def lerp(initial, target, t):
    return (1 - t) * initial + t * target
