import sys
import rospy
import moveit_commander



class PickUpObject:
    """

    """
    def __init__(self, pos_pickup_object, pos_leave_object):
        moveit_commander.roscpp_initialize(sys.argv)

        self.pos_pickup_object = pos_pickup_object
        self.pos_leave_object = pos_leave_object

        name_space = ""
        robot_descr = "robot_description"
        group_name = "widowx_arm"

        # robot
        self.robot = moveit_commander.RobotCommander(ns=name_space, robot_description=robot_descr)
        # arm
        self.move_group = moveit_commander.MoveGroupCommander(group_name, ns=name_space, robot_description=robot_descr)
        #gripper
        group_gipper_name = "widowx_gripper"
        self.gripper_joint= "gripper_joint"
        self.move_group_gripper = moveit_commander.MoveGroupCommander(group_gipper_name, ns=name_space, robot_description=robot_descr)


    def get_planning_info(self):
        # Getting the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print ("============ Planning frame: %s" % planning_frame)

        # Getting the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print ("============ End effector link: %s" % eef_link)

        # Getting a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print ("============ Available Planning Groups:" + str(self.robot.get_group_names()))

        # Printing the entire state of the robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print()

        # Printing the current pose:
        current_pose = self.move_group.get_current_pose()
        print("Current pose")
        print("===========")
        print(current_pose)

        # Printing the goal tolerance:
        print("Goal tolerance")
        print("==============")
        print(self.move_group.get_goal_tolerance())


    def go_to_pose(self, pose):
        # Planning to pose goal
        self.move_group.set_pose_target(pose)
        plan = self.move_group.go(wait=True)
        # Calling stop() ensures that there is no residual movement
        self.move_group.stop()
        # Clearing targets after planning.
        self.move_group.clear_pose_targets()


    def move_gripper(self, cm):
        self.move_group_gripper.set_joint_value_target(self.gripper_joint, cm) 
        self.move_group_gripper.go()        

    def close_gripper(self):
        self.move_gripper(0.001)

    def open_gripper(self):
        self.move_gripper(0.0157)

    def run(self):
        self.go_to_pose(self.pos_pickup_object)
        self.close_gripper()
        self.go_to_pose(self.pos_leave_object)
        self.open_gripper()
        self.go_to_pose(self.pos_base)


    



    


