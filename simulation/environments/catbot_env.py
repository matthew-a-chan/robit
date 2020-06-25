from environments.basic_balance_env import BasicBalanceEnv
import robots


class CatbotEnv(BasicBalanceEnv):

  def __init__(self):
    self.robot = robots.catbot()
    super().__init__(robot=self.robot)

  def compute_reward(self):
    return 1
    #if self.robot.failed():
    #    return -100
    #return -10*self.robot.parts['torso'].get_linear_velocity()[1] + 0.1 - self.robot.energy*0.05
