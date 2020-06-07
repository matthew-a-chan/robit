from environments.catbot_env import CatbotEnv
import robots
from environments.base_scene import NoGravityScene

class IKEnv(CatbotEnv):
    def __init__(self):
        super().__init__()

    def create_scene(self, bullet_client):
        self.scene = NoGravityScene(bullet_client)
        return self.scene
