import gym
import pybullet_data


class Scene:

  def __init__(self, bullet_client):
    self._p = bullet_client
    self._p.setAdditionalSearchPath(pybullet_data.getDataPath())

    self.np_random, seed = gym.utils.seeding.np_random(None)

  def episode_restart(self, bullet_client):
    self._p.setTimeStep(0.01)

  def global_step(self):
    self._p.stepSimulation()


class FlatScene(Scene):
  loaded = False

  def episode_restart(self, bullet_client):
    Scene.episode_restart(self, bullet_client)
    self._p.setGravity(0, 0, -9.8)
    #self._p.setDefaultContactERP(0.9)
    if not self.loaded:
      self.groundID = self._p.loadURDF("plane.urdf")
      self.loaded = True


class NoGravityScene(Scene):
  loaded = False

  def episode_restart(self, bullet_client):
    Scene.episode_restart(self, bullet_client)
    self._p.setGravity(0, 0, 0)
    #self._p.setDefaultContactERP(0.9)
    if not self.loaded:
      self.loaded = True
