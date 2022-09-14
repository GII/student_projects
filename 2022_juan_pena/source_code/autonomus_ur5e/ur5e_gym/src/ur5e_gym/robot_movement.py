# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from enum import Flag
from turtle import position
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, DeleteModel, \
    SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, PoseStamped
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_commander import MoveGroupCommander
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from math import pi
from copy import deepcopy
import moveit_commander
import time
from tf_conversions import posemath, toMsg
import PyKDL
from threading import Timer
import fcl
import tf2_ros
import numpy as np


class SmartGrasper(object):
    """
    This is the helper library to easily access the different functionalities of the simulated robot
    from python.
    """

    __last_joint_state = None
    __current_model_name = ["obstacle_0","cylinder"]
    __path_to_models = "/home/juan/.gazebo/models/"
    # __path_to_models = "/home/juan/tf_ws/src/ur5e_2FG7_gripper/robot_description/urdf/objects"

    def __init__(self):
        """
        This constructor initialises the different necessary connections to the topics and services
        and resets the world to start in a good position.
        """
        # rospy.init_node("smart_grasper")

        self.__joint_state_sub = rospy.Subscriber("/joint_states", JointState,
                                                  self.__joint_state_cb, queue_size=1)

        rospy.wait_for_service("/gazebo/get_model_state", 10.0)
        rospy.wait_for_service("/gazebo/reset_world", 10.0)
        self.__reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.__get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        
        rospy.wait_for_service("/check_state_validity",10.0)
        self.__check_val_state = rospy.ServiceProxy('/check_state_validity',GetStateValidity)
        
        rospy.wait_for_service("/gazebo/pause_physics")
        self.__pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.__unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        rospy.wait_for_service("/controller_manager/switch_controller")
        self.__switch_ctrl = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        rospy.wait_for_service("/gazebo/set_model_configuration")
        self._set_model = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
        rospy.wait_for_service("/gazebo/set_model_state")
        self.__move_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        
        rospy.wait_for_service("/gazebo/delete_model")
        self.__delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.__spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        self.__spawn__urdf_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self._scene = moveit_commander.PlanningSceneInterface()

        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.__get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.__pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)
        self.robot_commander = moveit_commander.RobotCommander()
        self.arm_commander = MoveGroupCommander("arm",wait_for_servers=10)
        self.arm_commander.set_goal_tolerance(0.01)
        self.arm_commander.set_pose_reference_frame("base_link")
        self.arm_commander.set_end_effector_link("tcp")
        
        
        self.gripper_commander = MoveGroupCommander("gripper",wait_for_servers=10)
        

        self.__gripper_traj_client = SimpleActionClient("/gripper_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        self.__arm_traj_client = SimpleActionClient("/arm_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)

        if self.__gripper_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
            rospy.logfatal("Failed to connect to /gripper_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /gripper_controller/follow_joint_trajectory in 4sec.")

        if self.__arm_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
            rospy.logfatal("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")

        self.init_pos = {'shoulder_pan_joint':-1.57, 'shoulder_lift_joint':0.0,'elbow_joint':-1.57,
        		 'wrist_1_joint':0.0, 'wrist_2_joint':0.0,'wrist_3_joint':0.0}
        
        # self.send_command(self.init_pos, duration=0.2)
        

    def hard_reset_world(self):
        """
        Resets the object poses in the world and the robot joint angles.
        """
        self.__switch_ctrl.call(start_controllers=[],
                                stop_controllers=["gripper_controller", "arm_controller", "joint_state_controller"],
                                strictness=SwitchControllerRequest.BEST_EFFORT)
        self.__pause_physics.call()

        joint_names = ['elbow_joint','left_finger_joint','shoulder_lift_joint','shoulder_pan_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint' ]
        #showlder pan a 0
        joint_positions = [-1.57, 0.0, 0.0, -1.57, 1.2, 0.0, 0.0]


        self._set_model.call(model_name="ur5e_2FG7",
                              urdf_param_name="robot_description",
                              joint_names=joint_names,
                              joint_positions=joint_positions)
 

        

        self.__unpause_physics.call()

        self.__reset_world()
        timer = Timer(0.0, self.__start_ctrl)
        timer.start()

        time.sleep(0.1)



    def get_object_pose(self,object_name = "cylinder"):
        """
        Gets the pose of the object specified relative to base_link.
        @return The pose of the object.
        """
        is_tf_ready = self._tf_buffer.can_transform("base_link","world",rospy.Time(),timeout=rospy.Duration(5.0))
        base_pose=Pose()
        if  is_tf_ready:
            trans = self._tf_buffer.lookup_transform("base_link","world", rospy.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            orientation=[]
            orientation.append(trans.transform.rotation.x)  
            orientation.append(trans.transform.rotation.y) 
            orientation.append(trans.transform.rotation.z) 
            orientation.append(trans.transform.rotation.w) 
        euler = euler_from_quaternion(orientation)
        object_pose = self.__get_pose_srv.call(object_name,"world",).pose
        transform = PyKDL.Frame(PyKDL.Rotation.RPY(euler[0],euler[1], euler[2]),
                        PyKDL.Vector(x, y, z))
        tip_pose_kdl = posemath.fromMsg(object_pose)
        final_pose = toMsg( transform * tip_pose_kdl )

        return final_pose

    def get_tip_pose(self):
        """
        Gets the current pose of the robot's tooltip in the world frame.
        @return the tip pose
        """
        
        tip_pose = self.arm_commander.get_current_pose("tcp").pose
       

        return tip_pose

    def move_tip_absolute(self, target):
        """
        Moves the tooltip to the absolute target in the base_link frame
        @param target is a geometry_msgs.msg.Pose
        @return True on success
        """
       
        self.arm_commander.set_start_state_to_current_state()        
        self.arm_commander.set_pose_targets([target])        
        plan = self.arm_commander.go(wait=True)   
        if not plan:
            return False        
        return True

    def move_tip(self, x=0., y=0., z=0., roll=0., pitch=0., yaw=0.):
        """
        Moves the tooltip in the world frame by the given x,y,z / roll,pitch,yaw.
        @return True on success
        """
        transform = PyKDL.Frame(PyKDL.Rotation.RPY(pitch, roll, yaw),
                                PyKDL.Vector(-x, -y, -z))

        tip_pose = self.get_tip_pose()
        tip_pose_kdl = posemath.fromMsg(tip_pose)
        final_pose = toMsg(tip_pose_kdl * transform)

        self.arm_commander.set_start_state_to_current_state()
        self.arm_commander.set_pose_targets([final_pose])
        plan = self.arm_commander.go(wait=True)        
        if not plan:
            return False
        return True

    def send_command(self, command, duration=0.2):
        """
        Send a dictionnary of joint targets to the arm and gripper directly.
        @param command: a dictionnary of joint names associated with a target:
                        {"left_finger_joint": -1.0, "shoulder_pan_joint": 1.0}
        @param duration: the amount of time it will take to get there in seconds. Needs to be bigger than 0.0
        """

        gripper_goal = None
        arm_goal = None
        arm_joints = []
        gripper_joint = []
        for joint, target in command.items():
            if "left_finger_joint" in joint:
                # if not gripper_goal:
                #     gripper_goal = FollowJointTrajectoryGoal()
                #     point = JointTrajectoryPoint()
                #     point.time_from_start = rospy.Duration.from_sec(duration)
                #     gripper_goal.trajectory.points.append(point)
                # gripper_goal.trajectory.joint_names.append(joint)
                # gripper_goal.trajectory.points[0].positions.append(target)
                # gripper_joint.append(target)
                gripper_goal=True
                self.gripper_commander.set_joint_value_target(joint,target)

            else:
                if not arm_goal:
                    arm_goal = FollowJointTrajectoryGoal()
                    point = JointTrajectoryPoint()
                    point.time_from_start = rospy.Duration.from_sec(duration)
                    arm_goal.trajectory.points.append(point)                    
                arm_goal.trajectory.joint_names.append(joint)
                arm_goal.trajectory.points[0].positions.append(target)
                arm_joints.append(target)
        if arm_goal:
            self.__arm_traj_client.send_goal(arm_goal)
            self.__arm_traj_client.wait_for_result()
            is_achieved=self.__arm_traj_client.get_result()
            if is_achieved == 0:
                is_achieved = True
            else:
                self.__arm_traj_client.cancel_all_goals()
                is_achieved = False
            #is_achieved = self.arm_commander.go(arm_joints,wait=True)
        elif gripper_goal:
            # self.__gripper_traj_client.send_goal_and_wait(gripper_goal)
            is_achieved = self.gripper_commander.go(wait=True)
        else:
            is_achieved = False

        return is_achieved

    def get_current_joint_state(self):
        """
        Gets the current state of the robot.
        @return joint positions, velocity and efforts as three dictionnaries
        """
        joints_position = {n: p for n, p in
                           zip(self.__last_joint_state.name,
                               self.__last_joint_state.position)}
        joints_velocity = {n: v for n, v in
                           zip(self.__last_joint_state.name,
                               self.__last_joint_state.velocity)}
        joints_effort = {n: v for n, v in
                         zip(self.__last_joint_state.name,
                             self.__last_joint_state.effort)}
        return joints_position, joints_velocity, joints_effort

    def open_gripper(self):
        """
        Opens the gripper.
        @return True on success
        """
        self.gripper_commander.set_named_target("open")
        plan = self.gripper_commander.go(wait=True)
        if not plan:
            return False
        return True
    def move_model(self,model_name="cylinder",x=0.0,y=0.0,z=0.0):
        """
        Moves a gazebo model relative to world
  
        """
        state =ModelState()
        state.model_name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1
        state.reference_frame="world"
        try:
            self.__move_model.call(state)
            flag = True
        except:
            rospy.logwarn("error trying to spawn the object")
            flag= False
        return flag

    def close_gripper(self):
        """
        Closes the gripper.
        @return True on success
        """
        self.gripper_commander.set_named_target("closed")
        plan = self.gripper_commander.go(wait=True)
        self.gripper_commander.stop()
        if not plan:
            return False
        return True
        

    def enable_finger_collisions(self, enable=True, objects=["cylinder_link","action_table","mytable"]):
        """
        Disables or enables the collisions check between the fingers and the objects / table
        @param enable: set to True to enable / False to disable
        @param objects: a list of objects objects to check collision with fingers
        @return True on success
        """      
        

        while self.__pub_planning_scene.get_num_connections() < 1:
            rospy.loginfo("waiting for someone to subscribe to the /planning_scene")
            rospy.sleep(0.1)

        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = self.__get_planning_scene(request)
        self.scene = response.scene
        
        acm = response.scene.allowed_collision_matrix

        for object_name in objects:
            if object_name not in acm.entry_names:
                # add object to allowed collision matrix
                acm.entry_names += [object_name]
                for row in range(len(acm.entry_values)):
                    acm.entry_values[row].enabled += [False]
                new_row = deepcopy(acm.entry_values[0])
                acm.entry_values.append(new_row) 

        for index_entry_values, entry_values in enumerate(acm.entry_values):
            if "finger" in acm.entry_names[index_entry_values]:                
                for index_value, _ in enumerate(entry_values.enabled):
                    if acm.entry_names[index_value] in objects:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
            elif acm.entry_names[index_entry_values] in objects:
                for index_value, _ in enumerate(entry_values.enabled):
                    if "finger" in acm.entry_names[index_value]:
                        if enable:
                            acm.entry_values[index_entry_values].enabled[index_value] = False
                        else:
                            acm.entry_values[index_entry_values].enabled[index_value] = True
        planning_scene_diff = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        self.__pub_planning_scene.publish(planning_scene_diff)
        rospy.sleep(1.0)
        return True
    def check_fingers_collision(self,object="cylinder"):
        """
        check if a object is in collision with the fingers
        @param object: name of the link you want to validate its collision
        """
        
        finger = fcl.Box(0.0192,0.032,0.045)
        collision_object = fcl.Cylinder(0.04,0.08)
        

        try:
            self._tf_buffer.can_transform("world","right_finger_v1_1",rospy.Time(),timeout=rospy.Duration(5.0))
            self._tf_buffer.can_transform("world","left_finger_v1_1",rospy.Time(),timeout=rospy.Duration(5.0))
        except:
            rospy.logwarn("tf is not ready")

        right_finger_trans=self._tf_buffer.lookup_transform("world","right_finger_v1_1", rospy.Time())       
        right_finger_t = np.array([right_finger_trans.transform.translation.x, right_finger_trans.transform.translation.y,\
                                    right_finger_trans.transform.translation.z])
        right_finger_q = np.array([right_finger_trans.transform.rotation.x, right_finger_trans.transform.rotation.y,\
                        right_finger_trans.transform.rotation.z, right_finger_trans.transform.rotation.w])
        right_finger_transformation = fcl.Transform(right_finger_q,right_finger_t)                
        right_finger_object = fcl.CollisionObject(finger,right_finger_transformation) 

        left_finger_trans=self._tf_buffer.lookup_transform("world","left_finger_v1_1", rospy.Time())
        left_finger_t = np.array([left_finger_trans.transform.translation.x, left_finger_trans.transform.translation.y,\
                                left_finger_trans.transform.translation.z])
        left_finger_q = np.array([left_finger_trans.transform.rotation.x, left_finger_trans.transform.rotation.y,\
                        left_finger_trans.transform.rotation.z,left_finger_trans.transform.rotation.w])
        left_finger_transformation = fcl.Transform(left_finger_q,left_finger_t)                
        left_finger_object = fcl.CollisionObject(finger,left_finger_transformation) 

        collision_object_trans=self.get_object_pose(object)
        collision_object_t = np.array([collision_object_trans.position.x, collision_object_trans.position.y,\
                    collision_object_trans.position.z])
        collision_object_q = np.array([collision_object_trans.orientation.x, collision_object_trans.orientation.y,\
                    collision_object_trans.orientation.z,collision_object_trans.orientation.w])

        collision_object_transformation = fcl.Transform(collision_object_q,collision_object_t)
        collision_object_object = fcl.CollisionObject(collision_object,collision_object_transformation)
        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        right_ret = fcl.collide(collision_object_object, right_finger_object, request, result)
        left_ret = fcl.collide(collision_object_object, left_finger_object, request, result)
       
        if right_ret >= 1 and left_ret >=1 :
            is_collided = True
        else:
            is_collided = False

        return is_collided
    def check_two_objects_collision(self,object_1_name="",object_2_name=""):
        """
        Retruns true on collision of two objects.
        """
        collision_object_1 = fcl.Cylinder(0.02,0.29)
        collision_object_2 = fcl.Cylinder(0.02,0.29)
        collision_object_1_trans=self.get_object_pose(object_1_name)
        collision_object_1_t = np.array([collision_object_1_trans.position.x, collision_object_1_trans.position.y,\
                    collision_object_1_trans.position.z])
        collision_object_1_q = np.array([collision_object_1_trans.orientation.x, collision_object_1_trans.orientation.y,\
                    collision_object_1_trans.orientation.z,collision_object_1_trans.orientation.w])
        collision_object_1_transformation = fcl.Transform(collision_object_1_q,collision_object_1_t)
        collision_object_1_object = fcl.CollisionObject(collision_object_1,collision_object_1_transformation)
        print("object _ 1 trans")
        print(collision_object_1_t)

        collision_object_2_trans=self.get_object_pose(object_2_name)
        collision_object_2_t = np.array([collision_object_2_trans.position.x, collision_object_2_trans.position.y,\
                    collision_object_2_trans.position.z])
        collision_object_2_q = np.array([collision_object_2_trans.orientation.x, collision_object_2_trans.orientation.y,\
                    collision_object_2_trans.orientation.z,collision_object_2_trans.orientation.w]) 
        collision_object_2_transformation = fcl.Transform(collision_object_2_q,collision_object_2_t)
        collision_object_2_object = fcl.CollisionObject(collision_object_2,collision_object_2_transformation)
        print("object _ 2 trans")
        print(collision_object_2_t)
        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        ret = fcl.collide(collision_object_1_object, collision_object_2_object, request, result)
        if ret >= 1 :
            is_collided = True
        else:
            is_collided = False
        return is_collided           

    def pick(self):
        """
        Does its best to pick the ball.
        """
        is_wrong= None
        rospy.loginfo("Moving to Pregrasp")
        is_wrong = self.open_gripper()
        time.sleep(0.1)

        ball_pose = self.get_object_pose("cylinder")
        ball_pose.position.z += 0.015
        ball_pose.position.x -= 0.14

        # setting an absolute orientation (from the top)
        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        ball_pose.orientation.x = quaternion[0]
        ball_pose.orientation.y = quaternion[1]
        ball_pose.orientation.z = quaternion[2]
        ball_pose.orientation.w = quaternion[3]

        self.move_tip_absolute(ball_pose)
        time.sleep(0.1)

        rospy.loginfo("Grasping")
        self.move_tip(y=-0.164)
        time.sleep(0.1)
        self.enable_finger_collisions(False)
        # time.sleep(0.1)
        self.close_gripper()
        time.sleep(0.1)
        

        rospy.loginfo("Lifting")
        for _ in range(5):
            self.move_tip(y=0.01)
            time.sleep(0.1)

        self.open_gripper()
        time.sleep(0.1)

       

    def swap_object(self,x=0.0,y=0.0,z=0.0,old_model_name = "cylinder",new_model_name="cylinder"):
        """
        Replaces the current object with a new one.Replaces
        @new_model_name the name of the folder in which the object is (e.g. beer)
        """
        try:
            self.__delete_model(old_model_name)
        except:
            rospy.logwarn("Failed to delete: " + old_model_name)
        try:
            sdf = None
            initial_pose = Pose()
            initial_pose.position.x = x
            initial_pose.position.y = y
            initial_pose.position.z = z
            initial_pose.orientation.w = 1
            with open(self.__path_to_models + new_model_name + "/model.sdf", "r") as model:
                sdf = model.read()
            res = self.__spawn_model(new_model_name, sdf, "", initial_pose, "world")
            rospy.logerr("RES: " + str(res))
            self.__current_model_name = new_model_name
        except:
            rospy.logwarn("Failed to delete: " + old_model_name)

    def spawn_object(self, new_model_name="cylinder",x=0.15,y=0.1,z=1.2,is_urdf=False):
        """
        Spawn a new urdf or sdf model, coordinates are relatives to the "world" link
        @new_model_name the name of the folder in which the object is (e.g. beer)
        """
        if is_urdf:
            try:
                urdf = None
                initial_pose = Pose()                
                initial_pose.position.x = x
                initial_pose.position.y=y
                initial_pose.position.z = z
                with open(self.__path_to_models + new_model_name + ".xacro") as model:
                    urdf = model.read()
                res = self.__spawn__urdf_model(new_model_name, urdf, "", initial_pose, "world")
                rospy.loginfo("RES: " + str(res))

            except:
                rospy.logwarn("Failed to spawn model: " )
        else:
            try:
                sdf = None
                initial_pose = Pose()
                initial_pose.position.x = x
                initial_pose.position.y =y
                initial_pose.position.z = z
                with open(self.__path_to_models + new_model_name + "/model.sdf", "r") as model:
                    sdf = model.read()
                res = self.__spawn_model(new_model_name, sdf, "", initial_pose, "world")
                # box_pose = PoseStamped()
                # box_pose.header.frame_id = "world"
                # # box_pose.pose.orientation.w = 1.0
                # box_pose.pose.position.x = x
                # box_pose.pose.position.y = y
                # box_pose.pose.position.z = z  # above the panda_hand frame
                # box_name = new_model_name
                # self._scene.add_cylinder(box_name,box_pose,0.05,0.02)
                
                rospy.loginfo("RES: " + str(res))
                # self.__current_model_name = new_model_name
            except:
                rospy.logwarn("Failed to spawn model: " )

    

    def __compute_arm_target_for_ball(self):
        ball_pose = self.get_object_pose()

        # come at it from the top
        arm_target = ball_pose
        arm_target.position.z += 0.5

        quaternion = quaternion_from_euler(-pi/2., 0.0, 0.0)
        arm_target.orientation.x = quaternion[0]
        arm_target.orientation.y = quaternion[1]
        arm_target.orientation.z = quaternion[2]
        arm_target.orientation.w = quaternion[3]

        return arm_target

    def __pre_grasp(self, arm_target):
        self.gripper_commander.set_named_target("open")
        plan = self.gripper_commander.plan()
        self.gripper_commander.execute(plan, wait=True)

        for _ in range(10):
            self.arm_commander.set_start_state_to_current_state()
            self.arm_commander.set_pose_targets([arm_target])
            plan = self.arm_commander.plan()
            if self.arm_commander.execute(plan):
                return True

    def __grasp(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z -= 0.12
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print (fraction)
        if not self.arm_commander.execute(plan):
            return False

        self.gripper_commander.set_named_target("close")
        plan = self.gripper_commander.plan()
        if not self.gripper_commander.execute(plan, wait=True):
            return False

        self.gripper_commander.attach_object("cricket_ball__link")

    def __lift(self, arm_target):
        waypoints = []
        waypoints.append(self.arm_commander.get_current_pose(self.arm_commander.get_end_effector_link()).pose)
        arm_above_ball = deepcopy(arm_target)
        arm_above_ball.position.z += 0.1
        waypoints.append(arm_above_ball)

        self.arm_commander.set_start_state_to_current_state()
        (plan, fraction) = self.arm_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        print (fraction)
        if not self.arm_commander.execute(plan):
            return False

    def __start_ctrl(self):
        rospy.loginfo("STARTING CONTROLLERS")
        self.__switch_ctrl.call(start_controllers=["gripper_controller", "arm_controller", "joint_state_controller"],
                                stop_controllers=[], strictness=1)

    def __joint_state_cb(self, msg):
        self.__last_joint_state = msg