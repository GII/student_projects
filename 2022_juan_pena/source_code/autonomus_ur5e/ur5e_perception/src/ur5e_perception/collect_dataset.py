"""Collect images of the picking task for training the autoencoder."""

import argparse
import os
import pickle
import rospy
import rospkg
import random
from geometry_msgs.msg import Pose
from math import pi,sqrt
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import cv2 as cv
from stable_baselines3.common.evaluation import evaluate_policy
import gym
import numpy as np
from tqdm import tqdm
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from ur5e_gym import reach_task_v3
from ur5e_gym.robot_movement import SmartGrasper

class ImageCollector():
    def __init__(self):
        

        # self.path=rospkg.get_ros_package_path()
        self.robot_movement = SmartGrasper()
        self.robot_movement.spawn_object(new_model_name="cylinder",x=0.8,y=0.0,z=1)
        self.robot_movement.spawn_object(new_model_name="marker_default",x=0.99,y=0.0,z=0.914)
        self.bridge=CvBridge()
        pack = rospkg.RosPack()
        package_path = pack.get_path('ur5e_perception')
        self.folder_path=package_path+"/data"
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self._camera_depth_image_raw_callback)        
        rospy.Subscriber("/camera/color/image_raw", Image, self._camera_rgb_image_raw_callback)
        # rospy.loginfo("Intiating environment")
        # self.env = gym.make('Ur5eReach-v3')
        # self.model = SAC("MlpPolicy", self.env, verbose=1,buffer_size=100)
        # rospy.loginfo("Env ready")


    def _move_env(self):
        robot_pose = Pose()
        x_cylinder = random.uniform(0.6, 0.965)
        y_cylinder = random.uniform(-0.5, 0.5)

        x_robot = random.uniform(0.25, 0.65)
        y_robot = random.uniform(-1*sqrt(abs(0.75**2 - x_robot**2 )), sqrt(abs(0.75**2 - x_robot**2 )))
        z_robot = random.uniform(sqrt(abs(0.75**2 - x_robot**2 -y_robot**2 ))- 0.15
                                , sqrt(abs(0.75**2 - x_robot**2 - y_robot**2 )))
        robot_pose.position.x = x_robot
        robot_pose.position.y = y_robot
        robot_pose.position.z = z_robot

        robot_pose.orientation.x = 0
        robot_pose.orientation.y = 0
        robot_pose.orientation.z = 0
        robot_pose.orientation.w = 1
        

        has_move_model = self.robot_movement.move_model(model_name="cylinder",x=x_cylinder,y=y_cylinder,z=0.914)
        has_move_model_2 = self.robot_movement.move_model(model_name="marker_default",x=x_cylinder +0.10,y=y_cylinder,z=0.914)

        has_moved_robot = self.robot_movement.move_tip_absolute(robot_pose)

        if has_move_model and has_move_model_2 and has_moved_robot:
            has_succeeded  = True
        else:
            has_succeeded = False
        return has_succeeded
        

        
    def collect_images_train(self,total_images=500):
        
        for image in tqdm(range(total_images)):

            flag = self._move_env()  
            color_path = self.folder_path+"/train/color/"
            depth_path =self.folder_path+"/train/depth/"
            os.makedirs(color_path, exist_ok=True)
            os.makedirs(depth_path, exist_ok=True)
            cv.imwrite(color_path+"image"+str(image)+".png",self.camera_rgb_image_raw )
            cv.imwrite(depth_path+"image"+str(image)+".png",self.camera_depth_image_raw )

    def collect_images_test(self,total_images=500):        
        for image in tqdm(range(total_images)):
            self._move_env()
            color_path = self.folder_path+"/test/color/"
            depth_path =self.folder_path+"/test/depth/"
            os.makedirs(color_path, exist_ok=True)
            os.makedirs(depth_path, exist_ok=True)
            cv.imwrite(color_path+"image"+str(image)+".png",self.camera_rgb_image_raw )
            cv.imwrite(depth_path+"image"+str(image)+".png",self.camera_depth_image_raw )

              


    def _camera_depth_image_raw_callback(self, data):

        cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        img_n = cv.normalize(src=cv_image, dst=None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
        img_n = 255- img_n
        im_color= cv.applyColorMap(img_n, cv.COLORMAP_JET)
        self.camera_depth_image_raw = im_color
            
    def _camera_rgb_image_raw_callback(self, data):
        self.camera_rgb_image_raw = self.bridge.imgmsg_to_cv2(data,desired_encoding="rgb8")  

