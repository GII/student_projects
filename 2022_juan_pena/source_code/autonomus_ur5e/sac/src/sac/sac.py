

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
from ur5e_gym import reach_task_v2
from ur5e_gym import reach_task_v3
from ur5e_gym import grab_and_lift_v1
from ur5e_gym import grab_and_lift_v0
from ur5e_gym import pusher_v0
from ur5e_gym import pusher_v1

from sensor_msgs.msg import Image
from stable_baselines3.common.monitor import Monitor
import gym
import numpy as np
from stable_baselines3 import sac
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from geometry_msgs.msg import Pose
import os
import tf2_ros



def main(model_name): 

    rospy.init_node("Ur_learning")
    r = rospkg.RosPack()
    path = r.get_path('sac')
    
    log_dir = path + "/log"+"/pusher-v1"
    os.makedirs(log_dir, exist_ok=True)
    
    env = gym.make("Pusher-v1")
    env = Monitor(env, log_dir)
    rospy.loginfo("Gym environment done") #checked

    callback = SaveOnBestTrainingRewardCallback(check_freq=100, log_dir=log_dir)

    model = SAC("MlpPolicy", env, verbose=1,buffer_size=1000,device="cpu")
    
    model.learn(total_timesteps=4000000,log_interval=4,callback=callback)
    model.save(model_name)
    rospy.loginfo("learning completed")
    rospy.signal_shutdown("finished")
   
        



