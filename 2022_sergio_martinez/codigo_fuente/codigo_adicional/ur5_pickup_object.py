import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from onrobot_2fg7_msgs.msg import OnRobot2FG7Output


class PickUpObject:
    def __init__(self, pos_pickup_object, pos_leave_object, pos_base):
        moveit_commander.roscpp_initialize(sys.argv)

        self.pos_pickup_object = pos_pickup_object
        self.pos_leave_object = pos_leave_object
        self.pos_base = pos_base

        name_space = ""
        robot_descr = "robot_description"
        group_name = "manipulator"

        # robot
        self.robot = moveit_commander.RobotCommander(ns=name_space, robot_description=robot_descr)
        
        # arm
        self.move_group = moveit_commander.MoveGroupCommander(group_name, ns=name_space, robot_description=robot_descr)
        self.move_group.set_goal_tolerance(0.01)
        self.move_group.allow_replanning(True)

        # gripper
        self.gripper_pub = rospy.Publisher("onrobot_output", OnRobot2FG7Output, queue_size=1)


    def go_to_pose(self, x, y, z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.orientation.w = 1
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        # Planning to pose goal
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.go(wait=True)
        # self.move_group.execute(plan, wait=True)
        # Calling stop() ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

    def close_gripper(self):
        print("Closing gripper")
        command = OnRobot2FG7Output()
        command.rGFR = 20
        command.rGWD = 350
        command.rGSP = 10
        command.rCTR = 1
        self.gripper_pub.publish(command)

    def open_gripper(self):
        print("Openinig gripper")
        command = OnRobot2FG7Output()
        command.rGFR = 20
        command.rGWD = 730  ## closing: 350
        command.rGSP = 10
        command.rCTR = 1
        self.gripper_pub.publish(command)

    def run(self):
        self.go_to_pose(self.pos_pickup_object)
        self.close_gripper()
        self.go_to_pose(self.pos_leave_object)
        self.open_gripper()
        self.go_to_pose(self.pos_base)

