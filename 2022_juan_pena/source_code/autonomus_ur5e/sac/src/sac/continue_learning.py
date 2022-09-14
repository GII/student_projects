import gym
import numpy
import time
from gym.wrappers.monitoring import video_recorder
from stable_baselines3 import DQN, SAC, DDPG, TD3, HerReplayBuffer
from stable_baselines3.her import GoalSelectionStrategy
from sac.learn import SaveOnBestTrainingRewardCallback
# ROS packages required
import rospy
import rospkg
# import our training environment
from ur5e_gym import reach_task_v1
from ur5e_gym import reach_task_v0
from sensor_msgs.msg import Image
from stable_baselines3.common.monitor import Monitor
import gym
import numpy as np
from stable_baselines3 import sac
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from geometry_msgs.msg import Pose
import os
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
import tf2_ros
from typing import Callable
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.env_util import make_vec_env



def main(model_name): 

    rospy.init_node("Ur_learning")
    r = rospkg.RosPack()
    path = r.get_path('sac')

    log_dir = path + "/log"
    model_dir = log_dir+"/best_model.zip"
    os.makedirs(log_dir, exist_ok=True)
    env_id = "Ur5eReach-v0"
    num_cpu = 4

    env = gym.make('Ur5eReach-v0')
    
    # vec_env = make_vec_env(env_id, n_envs=num_cpu)
    env = Monitor(env, log_dir)
    rospy.loginfo("Gym environment done") #checked

    callback = SaveOnBestTrainingRewardCallback(check_freq=100, log_dir=log_dir)
    model = SAC.load(path=model_dir,env=env)

    model.learn(total_timesteps=3000000,log_interval=4,callback=callback)
    model.save(model_name)
   
        



