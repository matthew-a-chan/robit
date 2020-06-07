import os
import math
import numpy as np

import gym
from gym import spaces
from gym.utils import seeding

import pybullet
import pybullet_data

import robots
from environments.base_env import BaseBulletEnv
from environments.base_scene import *

import time

class BasicBalanceEnv(BaseBulletEnv):

    def __init__(self, robot, render=False):
        super().__init__(robot, render)
        self.camera_x = 0
        self.stateId = -1
        self._envStepCounter = 0

    def step(self, action):
        if self.isRender:
            time.sleep(0.01)
        self.robot.set_motors(action)
        self.scene.global_step()
        observation = self.robot.compute_observation()
        reward = self.compute_reward()
        done = self.compute_done()

        self._envStepCounter += 1

        return np.array(observation), reward, done, {'r':reward, 'l':done}


    def create_scene(self, bullet_client):
        self.scene = FlatScene(bullet_client)
        return self.scene

    def reset(self):
        if self.stateId >= 0:
            self._p.restoreState(self.stateId)

        initial_observation = BaseBulletEnv.reset(self)
        if self.stateId < 0:
            self.stateId=self._p.saveState()
        self._envStepCounter = 0

        return initial_observation

    def compute_reward(self):
        return self.robot.reward()# - abs(self.robot.energy) * 0.005


    def compute_done(self):
        return self._envStepCounter >= 1000 or self.robot.failed()
