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

from rl_algorithms.CMA.CMA import CMAEvolution


def main(argv):
    env_id = 'IKEnv-v0'
    env = gym.make(env_id)

    save_path = f'models/cma_{env_id}.pkl'

    if '-c' in argv:
        cmaes = CMAEvolution.load(save_path)
        cmaes.set_env(env)
    else:
        policy = TrotController()  #NeuralNet([env.observation_space.shape[0], 64, 64, env.action_space.shape[0]])
        cmaes = CMAEvolution(policy=policy, env=env, checkpoint_path=save_path, population_size=64)

    cmaes.evolve(verbose=True, processes=4)


if __name__ == "__main__":
    main(sys.argv)
