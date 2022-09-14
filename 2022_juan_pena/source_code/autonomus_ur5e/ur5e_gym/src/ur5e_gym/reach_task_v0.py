import imp
from matplotlib.patches import FancyArrow
import rospy 
import numpy as np  
from gym import spaces
from ur5e_gym.ur5e_env import Ur5eEnv
from gym.envs.registration import register
from geometry_msgs.msg import Vector3, Pose
from std_msgs.msg import Header
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from math import pi,sqrt
import random
import cv2
from collections import deque

timestep_limit_per_episode = 100 # Can be any Value

register(
        id='Ur5eReach-v0',
        entry_point='ur5e_gym.reach_task_v0:Ur5ePick',
        max_episode_steps=timestep_limit_per_episode,
    )

class Ur5ePick(Ur5eEnv):
    def __init__(self):
        """
        Make ShadowTc learn how pick up a ball
        """
        
        # We execute this one before because there are some functions that this
        # TaskEnv uses that use variables from the parent class, like the effort limit fetch.
        super(Ur5ePick, self).__init__()
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        
        
        # Only variable needed to be set here

        rospy.logdebug("Start ShadowTcGetBallEnv INIT...")
      


        self.action_space = spaces.Box(low=np.array([-1.0,-1.0,-1.0]), high=np.array([1.0,1.0,1.0]), dtype=np.float32)

        

        self.reward_range = (-np.inf, np.inf)
        


        self.acceptable_distance_to_ball = 0.010
        
        
        # We place the Maximum and minimum values of observations
        # TODO: Fill when get_observations is done.

        #We spawn the object
                

        self.movement_system.spawn_object(new_model_name="cylinder",x=0.8,y=0.0,z=1)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape= (13,), dtype=np.float64)
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        


        self.cumulated_steps = 0.0

        rospy.logdebug("END shadow_tcGetBallEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the UR5 arm to the initial position and the objects to the original position.
        """
        rospy.logdebug("START _set_init_pose...")

        x = random.uniform(0.6, 0.965)
        y = random.uniform(-1*sqrt(abs(0.75**2 - x**2 )), sqrt(abs(0.75**2 - x**2 )))        


        y_robot = random.uniform(-0.65,0.65)
        z_robot = random.uniform(0, sqrt(abs(0.75**2 - y_robot**2 )))
        

        self.movement_system.arm_commander.set_position_target([0.2,y_robot,z_robot])
        self.movement_system.arm_commander.set_start_state_to_current_state()
        self.movement_system.gripper_commander.set_start_state_to_current_state()
        is_ok_robot = False
        counter = 0 

        while not is_ok_robot:
            is_ok_robot=self.movement_system.arm_commander.go(wait=True)
            counter += 1
            if counter >= 10:                
        
                self.movement_system.arm_commander.set_joint_value_target(self.movement_system.init_pos)
                is_ok_robot = False
                joints = np.array(self.joint_states.position)
                if joints[0] == -1.5 and joints[3] == -1.5:
                    is_ok_robot = True
                while not is_ok_robot:
                    is_ok_robot = self.movement_system.arm_commander.go(wait=True)
                    self.movement_system.arm_commander.stop()
        
        is_ok_object = self.movement_system.move_model(x=x,y=y,z=1)  


        
        rospy.logdebug("END _set_init_pose...")
        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        rospy.logdebug("START TaskEnv _init_env_variables")
    


        self.success_movement_count  = 0
        self.cumulated_reward = 0.0
        self.object_pose = self.movement_system.get_object_pose(object_name="cylinder")
        tip_pose = self._get_tip_pose()
        self.init_distance = self.get_distance_from_point(self.object_pose.position, tip_pose.position)
        self.distance_buffer = deque([])
        self.distance_buffer.appendleft(self.init_distance) 
        rospy.logdebug("END TaskEnv _init_env_variables")
        
        

    def _set_action(self, action):
        """
        It sets the joints of shadow_tc based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """ 
        #xnormalized = a + ( ((x - xminimum) * (b - a)) / range of x)
        actions_unnormalized = []
        

        actions_unnormalized.append(( ( (action[0]+ 1.0) * (0.75))) / (2.0) )
        actions_unnormalized.append(-0.375 + ( ( (action[1]+ 1.0) * (0.75))) / (2.0) )
        actions_unnormalized.append(( ( (action[2]+ 1.0) * (0.75))) / (2.0) )

        __actions = np.array(actions_unnormalized,dtype=np.float64)     

        self.movement_system.arm_commander.set_position_target(__actions)
        self.movement_system.arm_commander.set_start_state_to_current_state()
        self.movement_system.gripper_commander.set_start_state_to_current_state()
        self._is_adequate_movement=self.movement_system.arm_commander.go(wait=True)        
        self.movement_system.arm_commander.stop()

        
    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        shadow_tcEnv API DOCS.
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")
        tip_pose = self._get_tip_pose()
        joints = np.array(self.joint_states.position)
        object_position = np.array([self.object_pose.position.x,self.object_pose.position.y,self.object_pose.position.z])
        tip_pose = np.array([tip_pose.position.x,tip_pose.position.y,tip_pose.position.z])
        _observation = [] 
        for joint in joints:
            _observation.append(joint/(2*pi))
        for coordinate in object_position:
            _observation.append(coordinate)
        for coordinate in tip_pose:
            _observation.append(coordinate)

        observation = np.array(_observation,dtype=np.float64)
                        
        rospy.logdebug("Observations ==>"+str(observation))
        rospy.logdebug("END Get Observation ==>")

        return observation
        

    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) Planning once
        2) The TCP to block distance is lower than a threshold ( it got to the place )
           and the the collisions in the figuers are true.
        3) Robot has unsuccessfully moved more than 2 times
        """
        tip_pose = self._get_tip_pose()
        distance_from_ball = self.get_distance_from_point(self.object_pose.position, tip_pose.position)
               
        
        if len(self.distance_buffer) >= 3:
            self.distance_buffer.pop()
            
        self.distance_buffer.appendleft(distance_from_ball) 
        
        if self.distance_buffer[0] > self.distance_buffer[1]: #and self.distance_buffer[1] > self.distance_buffer[-1]:
            is_diverging = True
            
        else:
            is_diverging = False
        
  

        if self._is_adequate_movement:            
            self.success_movement_count += 1
              
        has_reached_the_ball = self.reached_ball(  tip_pose.position,
                                                    self.object_pose.position,
                                                    self.acceptable_distance_to_ball,
                                                    )
        



        done = has_reached_the_ball or is_diverging or not self._is_adequate_movement 
       
        
        rospy.loginfo("#### IS DONE ? ####")
        rospy.loginfo("has_reached_the_ball ?= "+str(has_reached_the_ball))
        rospy.loginfo("has failed executed times: ")
        rospy.loginfo( self.distance_buffer)
        rospy.loginfo(" times.")
        rospy.loginfo("has moved without success "+str(self.success_movement_count)+" times")
        rospy.loginfo("done ?="+str(done))
        rospy.loginfo("#### #### ####")
        
        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the ball has increased or not.
        :return:
        """

        tcp_pose = self._get_tip_pose()        
        distance_from_ball = self.get_distance_from_point(self.object_pose.position, tcp_pose.position)
        distance_reward = - distance_from_ball
        error_reward = self.init_distance / -2.0
        

        if self._is_adequate_movement:
            reward = distance_reward 
        else:
            reward = error_reward


        reward = distance_reward 
       
        self.cumulated_reward += reward
        rospy.loginfo("#### #### ####")
        rospy.loginfo("current reward " + str(reward) )
        rospy.loginfo("#### #### ####")
        self.cumulated_steps += 1
   

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
    

