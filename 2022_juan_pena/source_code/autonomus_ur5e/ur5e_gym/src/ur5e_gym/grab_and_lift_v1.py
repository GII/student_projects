from sqlite3 import enable_shared_cache
import rospy 
import numpy as np  
from gym import spaces
from ur5e_gym.ur5e_env import Ur5eEnv
from gym.envs.registration import register
from geometry_msgs.msg import Vector3, Pose
from tf.transformations import quaternion_from_euler
from math import pi,sqrt
import random
import cv2
from collections import deque

timestep_limit_per_episode = 20 # Can be any Value

register(
        id='GrabAndLift-v1',
        entry_point='ur5e_gym.grab_and_lift_v1:Ur5ePick',
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
        #things to add to the config file (workspace in absolute x y z r p y open close )

        self.action_space = spaces.Box(low=np.array([-1,-1,-1,-1,-1,-1,-1]),
                             high=np.array([1,1,1,1,1,1,1]), dtype=np.float32)

        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-np.inf, np.inf)
        


        self.acceptable_distance_to_ball = 0.05
        
        
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
        # We set the angles to zero of the limb

        x = 0.8
        y = 0.0
        z=1.15
        is_ok_object = self.movement_system.move_model(x=x,y=y,z=1)

        init_pose = Pose()
        init_pose.orientation.x = 0.7071068
        init_pose.orientation.z = 0.00000
        init_pose.orientation.y = 0.000
        init_pose.orientation.w = 0.7071068
        init_pose.position.x = x - 0.30
        init_pose.position.y = y
        init_pose.position.z = 1
    
        self.movement_system.arm_commander.set_end_effector_link("ee_link")
        self.movement_system.arm_commander.set_pose_reference_frame("world")
        self.movement_system.arm_commander.set_pose_targets([init_pose])  
        is_ok_robot = False
        while not is_ok_robot:      
            is_ok_robot = self.movement_system.arm_commander.go(wait=True)
            self.movement_system.arm_commander.stop()

        self.movement_system.arm_commander.set_pose_reference_frame("base_link")
        
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

        self.target_pose = Pose()
        self.target_pose.position.x = self.init_object_pose.position.x
        self.target_pose.position.y = self.init_object_pose.position.y
        self.target_pose.position.z = self.init_object_pose.position.z + 0.2

        tip_pose = self._get_tip_pose()
        self.init_distance = self.get_distance_from_point(self.init_object_pose.position, tip_pose.position)
            
        self.object_distance_buffer = deque([])
        self.object_distance_buffer.appendleft(self.init_distance) 

        self.target_distance_buffer = deque([])
        self.target_distance_buffer.appendleft(0.2) 
 
       

        rospy.logdebug("END TaskEnv _init_env_variables")
        
        

    def _set_action(self, action):
        """
        It sets the joints of shadow_tc based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """

        positions = self.joint_states
        positions = list(positions.position)

        #normalized = a + ( ((x - xminimum) * (b - a)) / range of x)
        actions_unnormalized = []

        for command in action:
            actions_unnormalized.append( -0.0174533 + ( ( (command + 1) * (0.03490664) ) ) / ( 2 ) )

        actions_unnormalized[-1] =  (0.0 + ( ( (command + 1) * (0.019) ) ) / ( 2 ) )

        _actions = np.array(actions_unnormalized,dtype=np.float64)

        arm_command ={'shoulder_pan_joint': positions[3]+_actions[0], 'shoulder_lift_joint':positions[2]+_actions[1],
                    'elbow_joint':positions[0]+_actions[2],'wrist_1_joint':positions[4]+_actions[3],
                     'wrist_2_joint':positions[5]+_actions[4],'wrist_3_joint':positions[6]+_actions[5]}
      
        #gripper max 0.019
        gripper_command = {"left_finger_joint":_actions[6]}
 
        
        self.movement_system.gripper_commander.set_joint_value_target(gripper_command)
        self.movement_system.arm_commander.set_joint_value_target(arm_command)
        gripper = self.movement_system.gripper_commander.go(wait=True)
        arm = self.movement_system.arm_commander.go(wait=True)
        self.movement_system.arm_commander.stop()
        self.movement_system.gripper_commander.stop()
        self._is_adequate_movement  = gripper and arm


        
        

        

            


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
        object_position = np.array([self.init_object_pose.position.x,self.init_object_pose.position.y,self.init_object_pose.position.z])
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
        1) Planning fails more than 3 times.
        2) The block is within 5 cm of the desired height
        3) Robot has unsuccessfully moved more than 15 times
        """
        tip_pose = self._get_tip_pose()
        has_collided = self.movement_system.check_fingers_collision(object="cylinder")



        if not has_collided:
            
            distance_from_ball = self.get_distance_from_point( self.init_object_pose.position, tip_pose.position)               
        
            if len(self.object_distance_buffer) >= 3:
                self.object_distance_buffer.pop()
            
            self.object_distance_buffer.appendleft(distance_from_ball) 
        
            if self.object_distance_buffer[0] > self.object_distance_buffer[1] and self.object_distance_buffer[1] > self.object_distance_buffer[-1]:
                is_diverging = True                
            else:
                is_diverging = False
            has_reached_taget = False
            
        else:

            distance_from_target = self.get_distance_from_point(tip_pose.position,self.target_pose)

            if len(self.target_distance_buffer) >= 3:
                self.target_distance_buffer.pop()
            self.target_distance_buffer.appendleft(distance_from_target)

            if self.target_distance_buffer[0] > self.target_distance_buffer[1] and self.target_distance_buffer[1] > self.target_distance_buffer[-1] :
                is_diverging = True
            else:
                is_diverging = False

            has_reached_taget = self.reached_ball(  tip_pose.position,
                                    self.target_pose.position,
                                    self.acceptable_distance_to_ball,
                                    )


       
              




        done = has_reached_taget or is_diverging or not self._is_adequate_movement 
            
        
        
        rospy.loginfo("#### IS DONE ? ####")
        rospy.loginfo("has_reached_the_ball ?= "+str(has_reached_taget))
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
        has_collided = self.movement_system.check_fingers_collision(object="cylinder")
        if not has_collided:

            tcp_pose = self._get_tip_pose() 
            object_pose = self.movement_system.get_object_pose(object_name="cylinder")       
            distance_from_ball = self.get_distance_from_point(object_pose.position, tcp_pose.position)
            distance_reward = - distance_from_ball

            left_finger_pose,right_finger_pose = self._get_fingers_pose()
            distance_from_left_finger  =  self.get_distance_from_point(object_pose.position, left_finger_pose.position)
            
            distance_from_right_finger =  self.get_distance_from_point(object_pose.position, right_finger_pose.position)
            
            fingers_reward = (-distance_from_left_finger) + ( -distance_from_right_finger)

            reward = fingers_reward + distance_reward
        else:

            target_distance = self.get_distance_from_point(object_pose.position, self.target_pose.position)
            target_reward = -target_distance

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
    

