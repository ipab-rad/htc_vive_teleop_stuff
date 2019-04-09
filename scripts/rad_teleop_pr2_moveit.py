#!/usr/bin/env python

import rospy
# TF stuff
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from math import radians
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


import threading
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# from trajectory_msgs.msg import *
# from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
# from sensor_msgs.msg import JointState
import actionlib

class PR2Gripper(object):
    """docstring for PR2Gripper"""
    def __init__(self, arm):
        super(PR2Gripper, self).__init__()
        self.arm = arm

        self.gripper_action_name = arm[0] + '_gripper_controller/gripper_action' #Specify [r]ight/[l]eft arm controller
        self.gripper_client = actionlib.SimpleActionClient(self.gripper_action_name, Pr2GripperCommandAction)

        if not self.gripper_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("ArmMover.py: right_gripper_client action server did not come up within timelimit")

    def open(self):
        return self.right_grip(0.08)

    def close(self):
        return self.right_grip(0, max_effort=100)

    def right_grip(self, position, max_effort = -1):
        goal = Pr2GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        return self.gripper_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(2.0),
                                                            preempt_timeout=rospy.Duration(0.1))


class PR2Teleop(object):
    def __init__(self):

	moveit_commander.roscpp_initialize(sys.argv)

	self.robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	self.group_left = moveit_commander.MoveGroupCommander("left_arm")
	self.group_right = moveit_commander.MoveGroupCommander("right_arm")

	rospy.sleep(2)

        self.last_left_pose = None
        self.left_pose = rospy.Subscriber('/left_controller_as_posestamped_offset',
                                          PoseStamped,
                                          self.left_cb, queue_size=1)
        self.last_right_pose = None
        self.right_pose = rospy.Subscriber('/right_controller_as_posestamped_offset',
                                           PoseStamped,
                                           self.right_cb, queue_size=1)
        # rostopic echo /vive_left
        self.last_left_buttons = None
        self.last_left_buttons_sub = rospy.Subscriber('/vive_left', Joy, 
                                        self.left_button_cb, queue_size=1)
        self.last_right_buttons = None
        self.last_right_buttons_sub = rospy.Subscriber('/vive_right', Joy, 
                                        self.right_button_cb, queue_size=1)

        self.left_gripper = PR2Gripper('left')
        self.right_gripper = PR2Gripper('right')

        self.left_vibrate_pub = rospy.Publisher('/vive_left_vibration', Float64, queue_size=1)
        self.right_vibrate_pub = rospy.Publisher('/vive_right_vibration', Float64, queue_size=1)

        self.msg_time_from_start_left = 0.8 # was 0.4
        self.msg_time_from_start_right = 0.8 # was 0.4

        self.move_eps = 0.1 # The arms are not going to move if the button is pressed less than this
        self.move_time_min = 0.4
        self.move_time_max = 8.0

        self.vibrate_right(3)
        # rospy.sleep(2.0)
        # self.left_gripper.open()
        # self.left_gripper.close()


    def vibrate_right(self, length=1, strength=0.5):
        def pub():
            msg_f = Float64()
            msg_f.data = strength

            rate = 50
            r = rospy.Rate(rate)
            for _ in range(length * rate):
                self.right_vibrate_pub.publish(msg_f)
                r.sleep()

        threading.Thread(target=pub).start()

    def vibrate_left(self, length=1, strength=0.5):
        def pub():
            msg_f = Float64()
            msg_f.data = strength

            rate = 50
            r = rospy.Rate(rate)
            for _ in range(length * rate):
                self.left_vibrate_pub.publish(msg_f)
                r.sleep()

        threading.Thread(target=pub).start()

    def left_cb(self, msg):
        self.last_left_pose = msg
        # print('Got a new left pose')

    def right_cb(self, msg):
        self.last_right_pose = msg
        # print('Got a new right pose')


    def left_button_cb(self, msg):
        self.last_left_buttons = msg
        # print(msg)
        if msg.buttons[2] == 1:
            if (msg.axes[1] > 0):
                self.left_gripper.close()
            else:
                self.left_gripper.open()
        if msg.buttons[3] == 1:
            self.vibrate_right(1)

    def right_button_cb(self, msg):
        self.last_right_buttons = msg
        # print(msg)
        if msg.buttons[2] == 1:
            if (msg.axes[1] > 0):
                self.right_gripper.close()
            else:
                self.right_gripper.open()
        if msg.buttons[3] == 1:
            self.vibrate_right(1)

    def calc_move_time(self, ratio):
        return self.move_time_min + \
                (self.move_time_max - self.move_time_min) / (1. - self.move_eps) * (1. - ratio)

    def right_hand_control(self):
        ps = self.last_right_pose
        if ps is None:
            # r.sleep()
            print("No last right pose...")
            return

        if self.last_right_buttons is None:
            print('No button information received (right).')
            return

        if self.last_right_buttons.axes[0] < self.move_eps:
            print('Right Button pressed too easy.')
            return

        x = self.last_right_pose.pose.position.x
        y = self.last_right_pose.pose.position.y
        z = self.last_right_pose.pose.position.z

        rx = self.last_right_pose.pose.orientation.x
        ry = self.last_right_pose.pose.orientation.y
        rz = self.last_right_pose.pose.orientation.z
        rw = self.last_right_pose.pose.orientation.w

	self.last_right_pose.header.frame_id = 'torso_lift_link'

        rospy.loginfo("Got pose: " + str(ps))

	self.group_right.clear_pose_targets()
	self.group_right.set_pose_target(self.last_left_pose.pose)
	plan_success = self.group_right.go(wait=True)
	print('Sending right pose sucess: ',plan_success)



    def left_hand_control(self):
        ps = self.last_left_pose
        if ps is None:
            # r.sleep()
            print("No last left pose...")
            return

        if self.last_left_buttons is None:
            print('No button information received (left).')
            return

        if self.last_left_buttons.axes[0] < self.move_eps:
            print('Left button pressed too easy.')
            return
	self.last_left_pose.header.frame_id = 'torso_lift_link'

        x = self.last_left_pose.pose.position.x
        y = self.last_left_pose.pose.position.y
        z = self.last_left_pose.pose.position.z

        rx = self.last_left_pose.pose.orientation.x
        ry = self.last_left_pose.pose.orientation.y
        rz = self.last_left_pose.pose.orientation.z
        rw = self.last_left_pose.pose.orientation.w

        rospy.loginfo("Got pose: " + str(ps))

	self.group_left.clear_pose_targets()
	self.group_left.set_pose_target(self.last_left_pose.pose)
	plan_success = self.group_left.go(wait=True)
	print('Sending left pose sucess: ',plan_success)



    def run(self, hand='both'):
        self.qinit_right = [0., 0., 0., 0., 0., 0., 0.] 
        self.qinit_left = [0., 0., 0., 0., 0., 0., 0.]
        # x = y = z = 0.0
        # rx = ry = rz = 0.0
        # rw = 1.0
        self.bx = self.by = self.bz = 0.01
        self.brx = self.bry = self.brz = 0.5

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if hand == 'right' or hand == 'both':
                self.right_hand_control()
            if hand == 'left' or hand == 'both':
                self.left_hand_control()

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('teleop_pr2')
    nv = PR2Teleop()
    nv.run(hand='both')
