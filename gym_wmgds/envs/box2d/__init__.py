try:
    import Box2D
    from gym_wmgds.envs.box2d.lunar_lander import LunarLander
    from gym_wmgds.envs.box2d.lunar_lander import LunarLanderContinuous
    from gym_wmgds.envs.box2d.bipedal_walker import BipedalWalker, BipedalWalkerHardcore
    from gym_wmgds.envs.box2d.car_racing import CarRacing
except ImportError:
    Box2D = None
