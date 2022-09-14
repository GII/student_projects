

import gym
import numpy
import time
import pandas as pd
from stable_baselines3 import  SAC

from sac.learn import SaveOnBestTrainingRewardCallback
# ROS packages required
import rospy
import rospkg
# import our training environment
from ur5e_gym import reach_task_v1
from ur5e_gym import reach_task_v0
from ur5e_gym import grab_and_lift_v1
from ur5e_gym import grab_and_lift_v0
from ur5e_gym import pusher_v0
from stable_baselines3.common.evaluation import evaluate_policy
from sensor_msgs.msg import Image
from stable_baselines3.common.monitor import Monitor
import gym
import numpy as np
from stable_baselines3 import sac
from geometry_msgs.msg import Pose
import os
import tf2_ros
import csv



def main(model_name="grabndlift-v0",num_episodes = 350,allowed_reward = -0.05): 
    rospy.init_node("model_tester")

    if "eacher" in model_name:
        if "0" in model_name:
            name = "Ur5eReach-v0"
        else:
            name = "Ur5eReach-v1"
    elif "usher" in model_name:
        name = "Pusher-v0"
    elif "ift" in model_name:
        if "0" in model_name:
            name = "GrabAndLift-v0"
        else:
            name = "GrabAndLift-v1"

    
    r = rospkg.RosPack()
    path = r.get_path('sac')
    
    log_dir = path + "/log"+"/"+model_name+"/test/random_object_and_random_robot_pose.csv"
    # os.makedirs(log_dir, exist_ok=True)


    env = gym.make(name)

    
  
    
    model = SAC.load(path+"/log/"+model_name+"/best_model.zip",env=env)
    # model = SAC.load("/home/juan/tfm_ws/src/autonomus_ur5e/sac/log/reacher-v0/best_model.zip")

    obs = env.reset()
    episodes = 0
    rewards = []
    success = []
    while not rospy.is_shutdown() and episodes < num_episodes:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        rewards.append(reward)
        if reward >= allowed_reward:
            success.append(1)
        else:
            success.append(0)

        if done:
            obs = env.reset()
        episodes += 1

    d = {
        "reward":rewards,
        "success":success
    } 
    df = pd.DataFrame(d)
    df.to_csv(log_dir)

    rospy.loginfo("testing completed")
    rospy.signal_shutdown("finished")
   
        



