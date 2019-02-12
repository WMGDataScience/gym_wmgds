from gym_wmgds.envs.toy_text.kellycoinflip import KellyCoinflipEnv


class TestKellyCoinflipEnv:
    @staticmethod
    def test_done_when_reaches_max_wealth():
        # https://github.com/openai/gym_wmgds/issues/1266
        env = KellyCoinflipEnv()
        env.seed(1)
        env.reset()
        done = False

        while not done:
            action = int(env.wealth * 20)  # bet 20% of the wealth
            observation, reward, done, info = env.step(action)

        assert env.wealth == env.max_wealth
