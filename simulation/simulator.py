#! /Users/matthewchan/Desktop/catbot3000/environment/bin/python3

import sys
sys.path.append('..')
import math

import ik

import gym
import time

import environments
import robots
import numpy as np
from trot_controller import TrotController


def gait_scheduler(timestep):
  max_timestep = 6000

  stride_length = 200
  stride_height = 30
  stride_width = -20
  swing_time = 2400

  stops = [0, 100, 150, swing_time, swing_time + 50, swing_time + 100, 6000]
  locations = np.array([[stride_width, -stride_length / 2, 0],
                        [stride_width, -stride_length / 2, stride_height],
                        [stride_width, -stride_length / 2, stride_height],
                        [stride_width, stride_length / 2 + stride_height, stride_height],
                        [stride_width, stride_length / 2 + stride_height, stride_height],
                        [stride_width, stride_length / 2, 0],
                        [stride_width, -stride_length / 2, 0]])

  x, y, z = np.interp(
      timestep, stops, locations[:, 0], period=max_timestep), np.interp(
          timestep, stops, locations[:, 1], period=max_timestep), np.interp(
              timestep, stops, locations[:, 2], period=max_timestep)
  return [x, y, z]


def controller(timestep, _):

  corrections = np.array([0, -.85, -1.55, 0, +.85, +1.55, 0, -.85, -1.55, 0, +.85, +1.55])

  center = np.array([0, 0, -200])
  offset = np.array([0, 0, 0])

  actions = np.zeros(12)

  speed = 80

  if True:
    offset = gait_scheduler(timestep * speed)
    actions[9:12] = ik.solve(center + offset, 'Front_Left')

    offset = gait_scheduler(timestep * speed + 3000)
    actions[6:9] = ik.solve(center + offset, 'Front_Right')

    offset = gait_scheduler(timestep * speed + 2000)
    actions[3:6] = ik.solve(center + offset, 'Back_Left')

    offset = gait_scheduler(timestep * speed + 5000)
    actions[0:3] = ik.solve(center + offset, 'Back_Right')

    #thing = actions.reshape ((4, 3))
    #print (thing)

  else:
    offset = gait_scheduler(timestep * speed)
    actions[9:12] = ik.solve(center + offset, 'Front_Left')

    offset = gait_scheduler(timestep * speed + 3000)
    offset[0] = -offset[0]
    actions[6:9] = ik.solve(center + offset, 'Front_Right')

    offset = gait_scheduler(timestep * speed + 3000)
    actions[3:6] = ik.solve(center + offset, 'Back_Left')

    offset = gait_scheduler(timestep * speed + 0)
    offset[0] = -offset[0]
    actions[0:3] = ik.solve(center + offset, 'Back_Right')

  actions = actions + corrections

  return actions


import pickle
tc = TrotController()
"""
with open('models/cma_IKEnv-v0.pkl', 'rb') as f:
    solution = pickle.load(f)['solution_mean']
    print(solution)
    tc.adopt_parameters(solution)
"""


def controller(timestep, observation):
  return tc.predict(observation)[0]


def main(argv):
  env_id = 'IKEnv-v0'
  env = gym.make(env_id)

  env.render('human')
  while True:
    done = False
    obs = env.reset()

    timesteps = 0
    total_reward = 0
    while not done:
      frame_start_time = time.time()
      action = controller(timesteps, obs)

      obs, rewards, done, info = env.step(action)
      env.render('human')
      total_reward += rewards
      delta_time = time.time() - frame_start_time
      if delta_time < 0.01:
        time.sleep(0.01 - delta_time)
      #time.sleep(0.01)
      timesteps += 1
    print(total_reward)


if __name__ == '__main__':
  main(sys.argv)
