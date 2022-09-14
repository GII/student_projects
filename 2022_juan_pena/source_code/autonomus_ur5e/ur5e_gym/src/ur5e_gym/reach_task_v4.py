from ntpath import join
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
import copy

timestep_limit_per_episode = 500 # Can be any Value

register(
        id='Ur5eReach-v4',
        entry_point='ur5e_gym.reach_task_v4:Ur5ePick',
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
      


        self.action_space = spaces.Box(low=np.array([-pi/2,-pi/2,-pi/2,-pi/2,-pi/2,-pi/2]),
                             high=np.array([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2]), dtype=np.float32)

        

        self.reward_range = (-np.inf, np.inf)
        


        self.acceptable_distance_to_ball = 0.09
        
        
        # We place the Maximum and minimum values of observations
        # TODO: Fill when get_observations is done.

        #We spawn the object
                

        self.movement_system.spawn_object(new_model_name="cylinder",x=0.8,y=0.0,z=1)
        self.observation_space = spaces.Box(low=np.array([-pi,-pi,-pi,-pi,-pi,-pi,0,0,0]),
                                    high=np.array([pi,pi,pi,pi,pi,pi,0.75,0.75,0.75]), dtype=np.float16)
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        
        self.done_reward =rospy.get_param("/shadow_tc/done_reward")
        self.closer_to_block_reward = rospy.get_param("/shadow_tc/closer_to_block_reward")

        self.cumulated_steps = 0.0

        rospy.logdebug("END shadow_tcGetBallEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the UR5 arm to the initial position and the objects to the original position.
        """
        rospy.logdebug("START _set_init_pose...")
        # We set the angles to zero of the limb

        x = random.uniform(0.6, 0.965)
        y = random.uniform(-1*sqrt(abs(0.75**2 - x**2 )), sqrt(abs(0.75**2 - x**2 )))
        is_ok_object = self.movement_system.move_model(x=x,y=y,z=1)       
        is_ok_robot = self.movement_system.send_command(self.movement_system.init_pos,duration=0.5)         


        
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
        self.ball_pose = self.movement_system.get_object_pose(object_name="cylinder")
        self.fail_buffer = deque([])


        rospy.logdebug("END TaskEnv _init_env_variables")
        
        

    def _set_action(self, action):
        """
        It sets the joints of shadow_tc based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """ 

        positions = self.joint_states
        positions = list(positions.position)
        _actions = np.array(action,dtype=np.float64)
        arm_command ={'shoulder_pan_joint': positions[3]+_actions[0], 'shoulder_lift_joint':positions[2]+_actions[1],
                    'elbow_joint':positions[0]+_actions[2],'wrist_1_joint':positions[4]+_actions[3],
                     'wrist_2_joint':positions[5]+_actions[4],'wrist_3_joint':positions[6]+_actions[5]}      


        arm = self.movement_system.send_command(arm_command) 
        self._is_adequate_movement = arm
        
    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        shadow_tcEnv API DOCS.
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")
        observation=[]
        joint_position=copy.deepcopy(self.joint_states)
        joint_position = list(joint_position.position)

        joint_position.pop(-1)

        if len(joint_position) <=5:
            joint_position=[-1.57, 0.0, 0.0, -1.57, 1.2, 0.0]

                
        for items in joint_position:
            observation.append(items)







        observation.append(self.ball_pose.position.x) 
        observation.append(self.ball_pose.position.y)
        observation.append(self.ball_pose.position.z)
 


                        
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
        tcp_pose = self._get_tip_pose()

        if len(self.fail_buffer) >= 5:
            self.fail_buffer.pop()
            
        self.fail_buffer.appendleft(self._is_adequate_movement) 
        sum_buffer = self.fail_buffer.count(False)

        if self._is_adequate_movement:            
            self.success_movement_count += 1
              
        has_reached_the_ball = self.reached_ball(  tcp_pose.position,
                                                    self.ball_pose.position,
                                                    self.acceptable_distance_to_ball,
                                                    )
        


        if (sum_buffer) >= 5:
            has_reached_fail_limit = True
        else:
            has_reached_fail_limit = False



        done = has_reached_the_ball or has_reached_fail_limit  
       
        
        rospy.loginfo("#### IS DONE ? ####")
        rospy.loginfo("has_reached_the_ball ?= "+str(has_reached_the_ball))
        rospy.loginfo("has failed executed times: "+str(sum_buffer)+" times")
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
        distance_from_ball = self.get_distance_from_point(self.ball_pose.position, tcp_pose.position)
        distance_reward = -10*distance_from_ball
        

        if self._is_adequate_movement:
            error_reward = 0.0
        else:
            error_reward = -10.0
    
        if done:
            reward = distance_reward + 100.0 

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
    

