from gym_wmgds.envs.mujoco.mujoco_env import MujocoEnv
# ^^^^^ so that user gets the correct error
# message if mujoco is not installed correctly
from gym_wmgds.envs.mujoco.ant import AntEnv
from gym_wmgds.envs.mujoco.half_cheetah import HalfCheetahEnv
from gym_wmgds.envs.mujoco.hopper import HopperEnv
from gym_wmgds.envs.mujoco.walker2d import Walker2dEnv
from gym_wmgds.envs.mujoco.humanoid import HumanoidEnv
from gym_wmgds.envs.mujoco.inverted_pendulum import InvertedPendulumEnv
from gym_wmgds.envs.mujoco.inverted_double_pendulum import InvertedDoublePendulumEnv
from gym_wmgds.envs.mujoco.reacher import ReacherEnv
from gym_wmgds.envs.mujoco.swimmer import SwimmerEnv
from gym_wmgds.envs.mujoco.humanoidstandup import HumanoidStandupEnv
from gym_wmgds.envs.mujoco.pusher import PusherEnv
from gym_wmgds.envs.mujoco.thrower import ThrowerEnv
from gym_wmgds.envs.mujoco.striker import StrikerEnv
