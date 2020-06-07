
import gym
#from stable_baselines import PPO2
#from stable_baselines.common.policies import MlpPolicy
#from stable_baselines.common import make_vec_env

import time

import environments
import robots
import sys
import numpy as np


def controller(timestep):
    hip_abductor = 0
    hip_forward = 0
    knee = 0


    return hip_abductor, hip_forward, knee


def main(argv):
    env_id = 'IKEnv-v0'
    env = gym.make(env_id)

    env.render('human')
    while True:
        done = False
        obs = env.reset()

        dead_counter = 0
        timesteps = 0
        while True:
            action = np.zeros(12)

            hip_abductor, hip_forward, knee = controller(timesteps)

            action[0] = hip_abductor
            action[1] = hip_forward
            action[2] = knee

            obs, rewards, done, info = env.step(action)
            env.render('human')
            time.sleep(0.01)
            timesteps += 1


if __name__ == '__main__':
    main(sys.argv)
