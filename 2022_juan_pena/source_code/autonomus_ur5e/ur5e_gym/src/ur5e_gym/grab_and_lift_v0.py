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

timestep_limit_per_episode = 5000 # Can be any Value

register(
        id='GrabAndLift-v0',
        entry_point='ur5e_gym.grab_and_lift_v0:Ur5ePick',
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

        self.action_space = spaces.Box(low=np.array([-1,-1,-1,-1]),
                             high=np.array([1,1,1,1]), dtype=np.float32)

        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-np.inf, np.inf)
        


        self.acceptable_distance_to_ball = 0.01
        
        
        # We place the Maximum and minimum values of observations
        # TODO: Fill when get_observations is done.

        #We spawn the object
                

        self.movement_system.spawn_object(new_model_name="cylinder",x=0.8,y=0.0,z=1)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape= (16,), dtype=np.float64)
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards

        self._is_random = False
        self.cumulated_steps = 0.0

        rospy.logdebug("END shadow_tcGetBallEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the UR5 arm to the initial position and the objects to the original position.
        """
        rospy.logdebug("START _set_init_pose...")
        # We set the angles to zero of the limb
        if self._is_random:
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

        else:
            x = 0.8
            y = 0.0
            is_ok_robot = False
            while not is_ok_robot:
                self.movement_system.arm_commander.set_joint_value_target(self.movement_system.init_pos)             
                is_ok_robot = self.movement_system.arm_commander.go(wait=True)
                self.movement_system.arm_commander.stop()



        is_ok_object = self.movement_system.move_model(model_name="cylinder",x=x,y=y,z=1)  
        rospy.logdebug("END _set_init_pose...")
        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        rospy.logdebug("START TaskEnv _init_env_variables")

        self._is_random = False
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

        
        actions_unnormalized = []

        # X action
        actions_unnormalized.append( 0 + ( ( (action[0] + 1) * (0.75) ) ) / ( 2 ) )
        # Y action
        actions_unnormalized.append( -0.375 + ( ( (action[1] + 1) * (0.75) ) ) / ( 2 ) )
        # Z action
        actions_unnormalized.append( 0 + ( ( (action[2]+ 1) * (0.75) ) ) / ( 2 ) )
        # RPY action                 
        actions_unnormalized.append(0.6532815)
        actions_unnormalized.append(0.6532815)
        actions_unnormalized.append(0.2705981)
        actions_unnormalized.append(0.2705981)
        # Gripper Action
        actions_unnormalized.append(0.0 + ( ( (action[-1] + 1) * (0.019) ) ) / ( 2 ) )

        _actions = np.array(actions_unnormalized,dtype=np.float64)

        gripper_command = {"left_finger_joint":_actions[-1]}

        
        self.movement_system.gripper_commander.set_joint_value_target(gripper_command)
        self.movement_system.arm_commander.set_pose_target(actions_unnormalized[:7])        
        arm = self.movement_system.arm_commander.go(wait=True)
        gripper = self.movement_system.gripper_commander.go(wait=True)
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
        object_pose = self.movement_system.get_object_pose(object_name="cylinder")

        object_position = np.array([object_pose.position.x,object_pose.position.y,object_pose.position.z])
        target_position = np.array([self.target_pose.position.x,self.target_pose.position.y,self.target_pose.position.z])
        tip_pose = np.array([tip_pose.position.x,tip_pose.position.y,tip_pose.position.z])
        _observation = [] 
        _observation.append(joints[0]/(pi))
        _observation.append(joints[1]/0.019)
        _observation.append(joints[2]/(2*pi))
        _observation.append(joints[3]/(2*pi))
        _observation.append(joints[4]/(2*pi))
        _observation.append(joints[5]/(2*pi))
        _observation.append(joints[6]/(2*pi))
       
        for coordinate in object_position:
            _observation.append(coordinate)
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


       
              

        if has_reached_taget:
            self._is_random = True


        done = has_reached_taget or is_diverging or not self._is_adequate_movement 
            
        
        
        rospy.loginfo("#### IS DONE ? ####")
        rospy.loginfo("has_reached_the_ball ?= "+str(has_reached_taget))
        rospy.loginfo("is divergin ?= "+str(is_diverging))
        rospy.loginfo("has moved with success? = "+str(self._is_adequate_movement))
        rospy.loginfo("done ?="+str(done))
        rospy.loginfo("#### #### ####")
        
        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the ball has increased or not.
        :return:
        """
        # has_collided = self.movement_system.check_fingers_collision(object="cylinder")
        tcp_pose = self._get_tip_pose() 
        object_pose = self.movement_system.get_object_pose(object_name="cylinder")       
        # distance_from_ball = self.get_distance_from_point(object_pose.position, tcp_pose.position)

        # has_reached_target = self.reached_ball(tcp_pose.position,object_pose.position,0.01)
        # #First 50% of the reward
        # if not has_reached_target :

        #     #normalized = a + ( ((x - xminimum) * (b - a)) / range of x)
        #     distance_reward =  -1 + ( ((distance_from_ball - 1.2)*(-0.5))/(1.2))

            
        #     reward = distance_reward 

        # elif has_reached_target and not has_collided : 

  

        #     reward = -0.5

        # elif has_reached_target and has_collided:

        #     target_distance = self.get_distance_from_point(object_pose.position, self.target_pose.position)
        #     target_reward = -0.3 + ( ((target_distance - 0.2)*(-0.3))/(0.2))
        #     reward = target_reward 
        
        # if not self._is_adequate_movement:
        #     reward -=  0.2
        distance_from_ball = self.get_distance_from_point(object_pose.position, tcp_pose.position)

        target_distance = self.get_distance_from_point(object_pose.position, self.target_pose.position)
        reward = -distance_from_ball - target_distance
        if not self._is_adequate_movement:
            reward =  (-self.init_distance - target_distance) * 3.5




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
    

