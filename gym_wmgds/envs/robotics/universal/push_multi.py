import os
from gym_wmgds import utils
from gym_wmgds.envs.robotics import universal_multi_env

import numpy as np

def to_radian(degree):
    return (degree*np.pi)/180.

# Ensure we get the path separator correct on windows
MODEL_XML_PATH = os.path.join('universal', 'push_multi.xml')

class UniversalPushMultiEnv(universal_multi_env.UniversalMultiEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse', n_objects=1, obj_action_type=[0,1,2], observe_obj_grp=False, obj_range=0.30):
        # initial_qpos_list = [360, 15, 180, 230, 360, 55, 90]
        initial_qpos_list = [0, 0, 0, 0, 0, 0, 0]

        initial_qpos = {
            #'robot0:slide0': 0.45,
            #'robot0:slide1': 0.75,
            #'robot0:slide2': 0.40,
            'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
            'object1:joint': [0.10, 0.025, 0.025, 1., 0., 0., 0.],
            'object2:joint': [0.20, 0.025, 0.025, 1., 0., 0., 0.],
            'object3:joint': [0.30, 0.025, 0.025, 1., 0., 0., 0.],
            'object4:joint': [0.40, 0.025, 0.025, 1., 0., 0., 0.],
            'object5:joint': [0.50, 0.025, 0.025, 1., 0., 0., 0.],
            'object6:joint': [0.60, 0.025, 0.025, 1., 0., 0., 0.],
            'object7:joint': [0.70, 0.025, 0.025, 1., 0., 0., 0.],
            'object8:joint': [0.80, 0.025, 0.025, 1., 0., 0., 0.],
            'object9:joint': [0.90, 0.025, 0.025, 1., 0., 0., 0.],
            #'robot0:shoulder_joint': to_radian(initial_qpos_list[0]),
            #'robot0:halfarm1_joint': to_radian(initial_qpos_list[1]),
            #'robot0:halfarm2_joint': to_radian(initial_qpos_list[2]),
            #'robot0:forearm_joint': to_radian(initial_qpos_list[3]),
            #'robot0:wrist1_joint': to_radian(initial_qpos_list[4]),
            #'robot0:wrist2_joint': to_radian(initial_qpos_list[5]),
            #'robot0:bracelet_joint': to_radian(initial_qpos_list[6])
        }
        universal_multi_env.UniversalMultiEnv.__init__(
            self, MODEL_XML_PATH, block_gripper=True, n_substeps=20,
            gripper_extra_height=0.0, target_in_the_air=False, target_stacked=False, target_offset=np.asarray([0.2,0,0]),
            obj_range=obj_range, target_range=0.15, distance_threshold=0.05,
            initial_qpos=initial_qpos, reward_type=reward_type, 
            n_objects=n_objects, obj_action_type=obj_action_type, observe_obj_grp=observe_obj_grp)
        utils.EzPickle.__init__(self)
