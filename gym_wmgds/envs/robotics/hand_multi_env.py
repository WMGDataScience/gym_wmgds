import os
import copy
import numpy as np

import gym_wmgds
from gym_wmgds import error, spaces
from gym_wmgds.utils import seeding
from gym_wmgds.envs.robotics import robot_env, utils


class HandMultiEnv(robot_env.RobotEnv):
    def __init__(self, model_path, n_substeps, initial_qpos, relative_control,n_actions):
        self.relative_control = relative_control
        self.n_actions = n_actions

        super(HandMultiEnv, self).__init__(
            model_path=model_path, n_substeps=n_substeps, n_actions=n_actions,
            initial_qpos=initial_qpos)

    # RobotEnv methods
    # ----------------------------

    def _set_action(self, action):

        assert action.shape == (self.n_actions,)
        action = action.copy()  # ensure that we don't change the action outside of this scope

        ctrlrange = self.sim.model.actuator_ctrlrange
        actuation_range = (ctrlrange[:, 1] - ctrlrange[:, 0]) / 2.
        if self.relative_control:
            actuation_center = np.zeros_like(action[:20])
            for i in range(self.sim.data.ctrl.shape[0]):
                actuation_center[i] = self.sim.data.get_joint_qpos(
                    self.sim.model.actuator_names[i].replace(':A_', ':'))
            for joint_name in ['FF', 'MF', 'RF', 'LF']:
                act_idx = self.sim.model.actuator_name2id(
                    'robot0:A_{}J1'.format(joint_name))
                actuation_center[act_idx] += self.sim.data.get_joint_qpos(
                    'robot0:{}J0'.format(joint_name))
        else:
            actuation_center = (ctrlrange[:, 1] + ctrlrange[:, 0]) / 2.
        self.sim.data.ctrl[:] = actuation_center + action[:20] * actuation_range
        self.sim.data.ctrl[:] = np.clip(self.sim.data.ctrl, ctrlrange[:, 0], ctrlrange[:, 1])

        # object actions
        action_obj = action[20:]
        action_obj = action_obj.reshape(self.n_objects, -1)
        
        obj_ctrl = np.concatenate((np.zeros((self.n_objects, 3)), 
                                    np.ones((self.n_objects, 1)), np.zeros((self.n_objects, 3))), axis=1)

        #for i_action in range(len(self.obj_action_type)):
        #    if self.obj_action_type[i_action] > 2:
        #        obj_ctrl[:,self.obj_action_type[i_action]] = action_obj[:,i_action] * 0.05
        #    else:
        #        obj_ctrl[:,self.obj_action_type[i_action]] = action_obj[:,i_action] * 0.05

        for i_action in range(len(self.obj_action_type)):
            obj_ctrl[:,self.obj_action_type[i_action]] = action_obj[:,i_action]

        if self.ai_object:
            obj_ctrl *= 0.05
        else:
            obj_ctrl *= 0.00

        action_obj = np.concatenate([obj_ctrl.ravel()])
    
        utils.mocap_set_action(self.sim, action_obj)

    def _viewer_setup(self):
        body_id = self.sim.model.body_name2id('robot0:palm')
        lookat = self.sim.data.body_xpos[body_id]
        for idx, value in enumerate(lookat):
            self.viewer.cam.lookat[idx] = value
        self.viewer.cam.distance = 0.5
        self.viewer.cam.azimuth = 55.
        self.viewer.cam.elevation = -25.
