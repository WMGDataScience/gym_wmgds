from gym_wmgds.envs.robotics.fetch_env import FetchEnv
from gym_wmgds.envs.robotics.fetch.slide import FetchSlideEnv
from gym_wmgds.envs.robotics.fetch.pick_and_place import FetchPickAndPlaceEnv
from gym_wmgds.envs.robotics.fetch.push import FetchPushEnv
from gym_wmgds.envs.robotics.fetch.reach import FetchReachEnv

#from gym_wmgds.envs.robotics.fetch.push_partial import FetchPushPartialEnv
#from gym_wmgds.envs.robotics.fetch.push_ld import FetchPushLDEnv

from gym_wmgds.envs.robotics.fetch_multi_env import FetchMultiEnv
from gym_wmgds.envs.robotics.fetch.push_multi import FetchPushMultiEnv
from gym_wmgds.envs.robotics.fetch.pick_and_place_multi import FetchPickAndPlaceMultiEnv
from gym_wmgds.envs.robotics.fetch.pick_and_place_multi import FetchPickAndPlaceFloorMultiEnv
from gym_wmgds.envs.robotics.fetch.stack_multi import FetchStackMultiEnv
from gym_wmgds.envs.robotics.fetch.stack_multi import FetchStackBordersMultiEnv
from gym_wmgds.envs.robotics.fetch.slide_multi import FetchSlideMultiEnv

from gym_wmgds.envs.robotics.hand.reach import HandReachEnv
from gym_wmgds.envs.robotics.hand.manipulate import HandBlockEnv
from gym_wmgds.envs.robotics.hand.manipulate import HandEggEnv
from gym_wmgds.envs.robotics.hand.manipulate import HandPenEnv

from gym_wmgds.envs.robotics.hand.manipulate_multi import HandBlockMultiEnv
from gym_wmgds.envs.robotics.hand.manipulate_multi import HandEggMultiEnv
from gym_wmgds.envs.robotics.hand.manipulate_multi import HandPenMultiEnv
from gym_wmgds.envs.robotics.hand.manipulate_multi import HandPenMultiShiftedEnv

from gym_wmgds.envs.robotics.hand.manipulate_touch_sensors import HandBlockTouchSensorsEnv
from gym_wmgds.envs.robotics.hand.manipulate_touch_sensors import HandEggTouchSensorsEnv
from gym_wmgds.envs.robotics.hand.manipulate_touch_sensors import HandPenTouchSensorsEnv

from gym_wmgds.envs.robotics.kortex_env import KortexEnv
from gym_wmgds.envs.robotics.kortex.reach import KortexReachEnv

from gym_wmgds.envs.robotics.kortex_multi_env import KortexMultiEnv
from gym_wmgds.envs.robotics.kortex.push_multi import KortexPushMultiEnv
#from gym_wmgds.envs.robotics.kortex.pick_and_place_multi import KortexPickAndPlaceMultiEnv

from gym_wmgds.envs.robotics.universal_multi_env import UniversalMultiEnv
from gym_wmgds.envs.robotics.universal.push_multi import UniversalPushMultiEnv
#from gym_wmgds.envs.robotics.kortex.pick_and_place_multi import KortexPickAndPlaceMultiEnv