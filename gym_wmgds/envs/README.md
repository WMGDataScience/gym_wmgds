# Envs

These are the core integrated environments. Note that we may later
restructure any of the files, but will keep the environments available
at the relevant package's top-level. So for example, you should access
`AntEnv` as follows:

```
# Will be supported in future releases
from gym_wmgds.envs import mujoco
mujoco.AntEnv
```

Rather than:

```
# May break in future releases
from gym_wmgds.envs.mujoco import ant
ant.AntEnv
```

## How to create new environments for gym_wmgds

* Create a new repo called gym_wmgds-foo, which should also be a PIP package.

* A good example is https://github.com/openai/gym_wmgds-soccer.

* It should have at least the following files:
  ```sh
  gym_wmgds-foo/
    README.md
    setup.py
    gym_wmgds_foo/
      __init__.py
      envs/
        __init__.py
        foo_env.py
        foo_extrahard_env.py
  ```

* `gym_wmgds-foo/setup.py` should have:

  ```python
  from setuptools import setup

  setup(name='gym_wmgds_foo',
        version='0.0.1',
        install_requires=['gym_wmgds']  # And any other dependencies foo needs
  )  
  ```

* `gym_wmgds-foo/gym_wmgds_foo/__init__.py` should have:
  ```python
  from gym_wmgds.envs.registration import register

  register(
      id='foo-v0',
      entry_point='gym_wmgds_foo.envs:FooEnv',
  )
  register(
      id='foo-extrahard-v0',
      entry_point='gym_wmgds_foo.envs:FooExtraHardEnv',
  )
  ```

* `gym_wmgds-foo/gym_wmgds_foo/envs/__init__.py` should have:
  ```python
  from gym_wmgds_foo.envs.foo_env import FooEnv
  from gym_wmgds_foo.envs.foo_extrahard_env import FooExtraHardEnv
  ```

* `gym_wmgds-foo/gym_wmgds_foo/envs/foo_env.py` should look something like:
  ```python
  import gym_wmgds
  from gym_wmgds import error, spaces, utils
  from gym_wmgds.utils import seeding

  class FooEnv(gym_wmgds.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
      ...
    def step(self, action):
      ...
    def reset(self):
      ...
    def render(self, mode='human', close=False):
      ...
  ```

## How to add new environments to gym_wmgds, within this repo (not recommended for new environments)

1. Write your environment in an existing collection or a new collection. All collections are subfolders of `/gym_wmgds/envs'.
2. Import your environment into the `__init__.py` file of the collection. This file will be located at `/gym_wmgds/envs/my_collection/__init__.py`. Add `from gym_wmgds.envs.my_collection.my_awesome_env import MyEnv` to this file.
3. Register your env in `/gym_wmgds/envs/__init__.py`:

 ```
register(
		id='MyEnv-v0',
		entry_point='gym_wmgds.envs.my_collection:MyEnv',
)
```

4. Add your environment to the scoreboard in `/gym_wmgds/scoreboard/__init__.py`:

 ```
add_task(
		id='MyEnv-v0',
		summary="Super cool environment",
		group='my_collection',
		contributor='mygithubhandle',
)
```
