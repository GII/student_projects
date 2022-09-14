#!/usr/bin/env python

import gym
import numpy
import time
from gym.wrappers.monitoring import video_recorder
# ROS packages required
import rospy
import rospkg
# import our training environment
from ur5e_gym import reach_task_v1
from ur5e_gym import reach_task_v0
from ur5e_gym import grab_and_lift_v0
from ur5e_gym import pusher_v0
from sensor_msgs.msg import Image
import gym
import numpy as np
from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from geometry_msgs.msg import Pose
import tf2_ros



def main(): 

    rospy.init_node("Ur_learning")
    
    env = gym.make('Pusher-v0')
    obs = env.reset()
    rospy.loginfo("Gym environment done") #checked

    # The noise objects for DDPG
    
    n_actions = env.action_space.shape[-1]
    
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
    

    model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1,buffer_size=1000)
    model.learn(total_timesteps=10000, log_interval=10)
    model.save("ddpg_ur5")
    env = model.get_env()

    del model # remove to demonstrate saving and loading

    model = DDPG.load("ddpg_ur5")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        



