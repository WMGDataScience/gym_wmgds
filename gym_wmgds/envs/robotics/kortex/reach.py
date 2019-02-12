from gym_wmgds import utils
from gym_wmgds.envs.robotics import kortex_env

import numpy as np

def to_radian(degree):
    return (degree*np.pi)/180.


class KortexReachEnv(kortex_env.KortexEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse'):
        # initial_qpos_list = [360, 15, 180, 230, 360, 55, 90]
        initial_qpos_list = [0, 0, 0, 0, 0, 0, 0]
        initial_qpos = {
             'robot0:slide0': 0.45,
             'robot0:slide1': 0.75,
             'robot0:slide2': 0.40,
             'robot0:shoulder_joint': to_radian(initial_qpos_list[0]),
             'robot0:halfarm1_joint': to_radian(initial_qpos_list[1]),
             'robot0:halfarm2_joint': to_radian(initial_qpos_list[2]),
             'robot0:forearm_joint': to_radian(initial_qpos_list[3]),
             'robot0:wrist1_joint': to_radian(initial_qpos_list[4]),
             'robot0:wrist2_joint': to_radian(initial_qpos_list[5]),
             'robot0:bracelet_joint': to_radian(initial_qpos_list[6])
         }
        # kortex_env.KortexEnv.__init__(
        #     self, 'kortex/reach.xml', has_object=False, block_gripper=True, n_substeps=20,
        #     gripper_extra_height=0.2, target_in_the_air=True, target_offset=0.0,
        #     obj_range=0.15, target_range=0.15, distance_threshold=0.05,
        #     initial_qpos=initial_qpos, reward_type=reward_type)
        kortex_env.KortexEnv.__init__(
            self, 'kortex/reach.xml', has_object=False, block_gripper=True, n_substeps=20,
            gripper_extra_height=0., target_in_the_air=True, target_offset=np.asarray([0.2,0,0]),
            obj_range=0.30, target_range=0.15, distance_threshold=0.05,
            initial_qpos=initial_qpos, reward_type=reward_type)
        utils.EzPickle.__init__(self)
