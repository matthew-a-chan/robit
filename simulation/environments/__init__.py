import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(id='basic_balance_env-v0', entry_point='environments.basic_balance_env:BasicBalanceEnv')
register(id='WiggleBotEnv-v0', entry_point='environments.wigglebot_env:WiggleBotEnv')
register(id='CatbotEnv-v0', entry_point='environments.catbot_env:CatbotEnv')
register(
    id='DoubleJointedBalanceBotEnv-v0',
    entry_point='environments.doublejointedbalancebot_env:DoubleJointedBalanceBotEnv')

register(id='CatbotMimicEnv-v0', entry_point='environments.catbot_mimic_env:CatbotMimicEnv')

register(id='IKEnv-v0', entry_point='environments.ik_env:IKEnv')
