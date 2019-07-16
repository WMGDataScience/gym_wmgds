import os
from gym_wmgds import utils
from gym_wmgds.envs.robotics import fetch_flex_env, fetch_flex_hinged_env


# Ensure we get the path separator correct on windows

class FetchPickAndPlaceFlexEnv(fetch_flex_env.FetchFlexEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse', n_objects=3, obj_action_type=[0,1,2], observe_obj_grp=False, obj_range=0.15):
        if n_objects == 3:
            MODEL_XML_PATH = os.path.join('fetch', 'pick_and_place_flex.xml')
            initial_qpos = {
                'robot0:slide0': 0.405,
                'robot0:slide1': 0.48,
                'robot0:slide2': 0.0,
                'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],        
                'object3:joint': [-0.350, 0.025, 0.025, 1., 0., 0., 0.],  
                }
        elif n_objects == 5:
            MODEL_XML_PATH = os.path.join('fetch', 'pick_and_place_flex_5.xml')
            initial_qpos = {
                'robot0:slide0': 0.405,
                'robot0:slide1': 0.48,
                'robot0:slide2': 0.0,
                'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],         
                'object5:joint': [-0.550, 0.025, 0.025, 1., 0., 0., 0.],  
                }
        fetch_flex_env.FetchFlexEnv.__init__(
            self, MODEL_XML_PATH, block_gripper=False, n_substeps=20,
            gripper_extra_height=0.2, target_in_the_air=False, target_stacked=False, target_offset=0.0,
            obj_range=obj_range, target_range=0.15, distance_threshold=0.025,
            initial_qpos=initial_qpos, reward_type=reward_type,
            n_objects=n_objects, obj_action_type=obj_action_type, observe_obj_grp=observe_obj_grp)
        utils.EzPickle.__init__(self)


class FetchPickAndPlaceFlexHingedEnv(fetch_flex_hinged_env.FetchFlexHingedEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse', n_objects=3, obj_action_type=[0,1,2], observe_obj_grp=False, obj_range=0.15):
        MODEL_XML_PATH = os.path.join('fetch', 'pick_and_place_flex_hinged_5.xml')
        initial_qpos = {
            'robot0:slide0': 0.405,
            'robot0:slide1': 0.48,
            'robot0:slide2': 0.0,
            'base0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],         
            'base5:joint': [-0.550, 0.025, 0.025, 1., 0., 0., 0.],
            }
        fetch_flex_hinged_env.FetchFlexHingedEnv.__init__(
            self, MODEL_XML_PATH, block_gripper=False, n_substeps=20,
            gripper_extra_height=0.2, target_in_the_air=False, target_stacked=False, target_offset=0.0,
            obj_range=obj_range, target_range=0.15, distance_threshold=0.025,
            initial_qpos=initial_qpos, reward_type=reward_type,
            n_objects=n_objects, obj_action_type=obj_action_type, observe_obj_grp=observe_obj_grp)
        utils.EzPickle.__init__(self)
