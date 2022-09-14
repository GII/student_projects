import rospy 
import numpy as np  
from gym import spaces
from ur5e_gym.ur5e_env import Ur5eEnv
from gym.envs.registration import register
from geometry_msgs.msg import Vector3, Pose
from math import pi,sqrt
import random
import copy
import cv2
from collections import deque

timestep_limit_per_episode = 2# Can be any Value

register(
        id='Pusher-v1',
        entry_point='ur5e_gym.pusher_v1:Ur5ePick',
        max_episode_steps=timestep_limit_per_episode,
    )

class Ur5ePick(Ur5eEnv):
    def __init__(self):
        """
        Ur5e learn how to push an object to a a point.
        """
        
        # We execute this one before because there are some functions that this
        # TaskEnv uses that use variables from the parent class, like the effort limit fetch.
        super(Ur5ePick, self).__init__()
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        
        
        # Only variable needed to be set here

        rospy.logdebug("Start ShadowTcGetBallEnv INIT...")
        #things to add to the config file (workspace in absolute x y z r p y open close )

        self.action_space = spaces.Box(low=np.array([-1]), high=np.array([1]), dtype=np.float32)
        
        self.observation_space = spaces.Box(low=0, high=1, shape= (200,), dtype=np.float32)

        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-np.inf, np.inf)
        


        self.acceptable_distance_to_ball = 0.01
        
        
        # We place the Maximum and minimum values of observations
        # TODO: Fill when get_observations is done.

        #We spawn the object
                

        self.movement_system.spawn_object(new_model_name="cylinder",x=0.8,y=0.0,z=1)
        self.movement_system.spawn_object(new_model_name="marker_default",x=0.95,y=0.0,z=0.914)
        
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        

        self.cumulated_steps = 0.0

        rospy.logdebug("END shadow_tcGetBallEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the UR5 arm to the initial position and the objects to the original position.
        """
        rospy.logdebug("START _set_init_pose...")
     




        
        rospy.sleep(0.2)
        self.movement_system.arm_commander.set_end_effector_link("ee_link")
        x_robot = random.uniform(0.54, 0.68)
        init_pose = Pose()
        #recto es todo con 0.5 y habria que mover un poco el cilindro hacia adelante 
        init_pose.orientation.x = 0.7071068
        init_pose.orientation.z = 0.00000
        init_pose.orientation.y = 0.00000
        init_pose.orientation.w = 0.7071068
        init_pose.position.x = random.uniform(0.54, 0.68)
        init_pose.position.y = 0.0
        init_pose.position.z = 0.96 
        self.movement_system.arm_commander.set_pose_reference_frame("world")
        self.movement_system.arm_commander.set_pose_targets([init_pose]) 
        self.check_init = False
        while not self.check_init:
            self.check_init = self.movement_system.arm_commander.go(wait=True)
            self.movement_system.open_gripper()
            self.movement_system.arm_commander.stop()

        self.movement_system.arm_commander.set_pose_reference_frame("base_link")
        is_ok_object = self.movement_system.move_model(x=0.8,y=0.0,z=1)
        is_ok_object = self.movement_system.move_model(model_name="marker_default",x=random.uniform(0.85, 0.97),y=0.0,z=0.914)
        
        rospy.logdebug("END _set_init_pose...")
        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        rospy.logdebug("START TaskEnv _init_env_variables")
    

        self.fail_movement_count = 0
        self.success_movement_count  = 0
        self.cumulated_reward = 0.0
        self.init_object_pose = self.movement_system.get_object_pose(object_name="cylinder")
        
        self.target_pose = self.movement_system.get_object_pose(object_name="marker_default")
        # self.target_pose.position.x = self.init_object_pose.position.x + 0.15
        # self.target_pose.position.y = self.init_object_pose.position.y
        # self.target_pose.position.z = self.init_object_pose.position.z 
    
     
       

        rospy.logdebug("END TaskEnv _init_env_variables")
        
        

    def _set_action(self, action):
        """
        It sets the joints of shadow_tc based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """
        #xnormalized = a + ( ((x - xminimum) * (b - a)) / range of x)
        actions_unnormalized =  0.05 + ( ( (action[0] + 1.0) * (0.25))) / (2.0) 
        waypoints = []
        partitions = 10.0
        wpose = Pose()
        self.tf_buffer.can_transform("base_link","ee_link",rospy.Time(),timeout=rospy.Duration(5.0))
        trans = self.tf_buffer.lookup_transform("base_link","ee_link", rospy.Time())
        wpose.position.x = trans.transform.translation.x
        wpose.position.y = trans.transform.translation.y
        wpose.position.z = trans.transform.translation.z
        wpose.orientation.x = trans.transform.rotation.x
        wpose.orientation.y =  trans.transform.rotation.y
        wpose.orientation.z = trans.transform.rotation.z
        wpose.orientation.w =  trans.transform.rotation.w

        pose_goal = Pose()
        pose_goal.orientation.x = wpose.orientation.x
        pose_goal.orientation.z = wpose.orientation.y
        pose_goal.orientation.y = wpose.orientation.z
        pose_goal.orientation.w = wpose.orientation.w
        pose_goal.position.x = wpose.position.x + 0.3

        x_delta = (pose_goal.position.x  - wpose.position.x) / partitions

        for i in range(int(partitions)):
            wpose.position.x += x_delta
            waypoints.append(copy.deepcopy(wpose))

        self.movement_system.arm_commander.set_end_effector_link("ee_link")
        (plan, fraction) = self.movement_system.arm_commander.compute_cartesian_path(waypoints,0.01,0.0) 
        self.movement_system.arm_commander.set_start_state_to_current_state()
        self.movement_system.gripper_commander.set_start_state_to_current_state()             
        self._is_adequate_movement=self.movement_system.arm_commander.execute(plan, wait=True)
        self.movement_system.arm_commander.stop()

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        shadow_tcEnv API DOCS.
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")


        color_features = list(self.color_features)
        depth_features = list(self.depth_features)    
        
        observation = np.array(color_features+depth_features,dtype=np.float32)
                        
        rospy.logdebug("Observations ==>"+str(observation))
        rospy.logdebug("END Get Observation ==>")

        return observation
        

    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) Planning fails more than 3 times.
        2) The block is within 5 cm of the desired height
        3) Robot has unsuccessfully moved more than 1 times
        """
            


       
       
        
 
        return True

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the ball has increased or not.
        :return:
        """
    
        object_pose = self.movement_system.get_object_pose(object_name="cylinder")       
        
        target_distance = self.get_distance_from_point(object_pose.position, self.target_pose.position)
        if self._is_adequate_movement:
            target_reward = -target_distance
        else :
            target_reward = -0.15

        reward = target_reward



       
        self.cumulated_reward += reward
        rospy.loginfo("#### #### ####")
        rospy.loginfo("current reward " + str(reward) )
        rospy.loginfo("#### #### ####")
        self.cumulated_steps += 1
        # rospy.loginfo("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods

    def reached_ball(self,tcp_position, ball_position, minimum_distance):
        """
        Return true if the distance from TCP position to the ball position is 
        lower than the minimum_distance and all three finguers are touching the ball.
        """
        
        distance_from_ball = self.get_distance_from_point(tcp_position, ball_position)
        
        distance_to_ball_ok = distance_from_ball < minimum_distance
        
        reached_ball_b = distance_to_ball_ok
        
        rospy.logdebug("###### REACHED BLOCK ? ######")
        rospy.logdebug("distance_from_ball==>"+str(distance_from_ball))
        rospy.logdebug("distance_to_ball_ok==>"+str(distance_to_ball_ok))
        rospy.logdebug("reached_ball_b==>"+str(reached_ball_b))
        rospy.logdebug("############")
        
        return reached_ball_b

        
    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = np.array((pstart.x, pstart.y, pstart.z))
        b = np.array((p_end.x, p_end.y, p_end.z))
    
        distance = np.linalg.norm(a - b)

    
        return distance
    

