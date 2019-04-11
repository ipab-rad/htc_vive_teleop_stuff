#!/usr/bin/env python

import argparse
import threading, subprocess, time
from math import radians

import rospy
import actionlib
# TF stuff
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

from trac_ik_python.trac_ik import IK
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal


class PR2Gripper(object):
    """docstring for PR2Gripper"""
    def __init__(self, arm):
        super(PR2Gripper, self).__init__()
        self.arm = arm

        self.gripper_action_name = arm[0] + '_gripper_controller/gripper_action' #Specify [r]ight/[l]eft arm controller
        self.gripper_client = actionlib.SimpleActionClient(self.gripper_action_name, Pr2GripperCommandAction)

        if not self.gripper_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("PR2Gripper: right_gripper_client action server did not come up within timelimit")

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
    def __init__(self, should_filter=True, 
                 button_start_script=None, 
                 button_stop_script=None):
        self.ik_right = IK("torso_lift_link",
                           "r_wrist_roll_link")
                           #"r_gripper_tool_frame")
        self.ik_left = IK("torso_lift_link",
                          "l_wrist_roll_link")

        if should_filter:
            should_filter_str='_filter'
        else:
            should_filter_str=''

        self.left_command = rospy.Publisher('/l_arm_controller/command'+should_filter_str,
                                            JointTrajectory,
                                            queue_size=1)

        self.right_command = rospy.Publisher('/r_arm_controller/command'+should_filter_str,
                                             JointTrajectory,
                                             queue_size=1)

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

        # self.msg_time_from_start_left = 0.8 # was 0.4
        # self.msg_time_from_start_right = 0.8 # was 0.4

        self.move_eps = 0.1 # The arms are not going to move if the button is pressed less than this
        self.move_time_min = 0.4
        self.move_time_max = 8.0

        self.episode_state = 'stopped'
        self.episode_state_change_time = time.time()
        self.episode_state_change_hist_time = 1
        self.button_start_script = button_start_script
        self.button_stop_script = button_stop_script

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
        # print('Button press msg: ', msg)
        if msg.buttons[2] == 1:
            if (msg.axes[1] > 0):
                self.left_gripper.close()
            else:
                self.left_gripper.open()
        if msg.buttons[3] == 1:
            self.vibrate_right(1)
        if msg.buttons[4] == 1:
            if time.time() - self.episode_state_change_time > self.episode_state_change_hist_time:
                # update last time command was executed!
                self.episode_state_change_time = time.time()

                if self.episode_state == 'stopped' and self.button_start_script is not None:
                    print('right', self.button_start_script)
                    print('right', self.button_stop_script)
                    cmd = subprocess.call(self.button_start_script)
                    self.episode_state = 'started'
                    self.vibrate_left(1, strength=0.6)
                elif self.episode_state == 'started' and self.button_stop_script is not None:
                    cmd = subprocess.call(self.button_stop_script)
                    self.episode_state = 'stopped'
                    self.vibrate_left(1, strength=0.1)
                else:
                    rospy.logerr('Error in grip button state!!!')


    def right_button_cb(self, msg):
        self.last_right_buttons = msg
        # print('Button press msg: ', msg)
        if msg.buttons[2] == 1:
            if (msg.axes[1] > 0):
                self.right_gripper.close()
            else:
                self.right_gripper.open()
        if msg.buttons[3] == 1:
            self.vibrate_right(1)
        if msg.buttons[4] == 1:
            if time.time() - self.episode_state_change_time > self.episode_state_change_hist_time:
                # update last time command was executed!
                self.episode_state_change_time = time.time()

                if self.episode_state == 'stopped' and self.button_start_script is not None:
                    print('right', self.button_start_script)
                    print('right', self.button_stop_script)
                    cmd = subprocess.call(self.button_start_script)
                    self.episode_state = 'started'
                    self.vibrate_left(1, strength=0.6)
                elif self.episode_state == 'started' and self.button_stop_script is not None:
                    cmd = subprocess.call(self.button_stop_script)
                    self.episode_state = 'stopped'
                    self.vibrate_left(1, strength=0.1)
                else:
                    rospy.logerr('Error in grip button state!!!')

    def calc_move_time(self, ratio):
        return self.move_time_min + \
                (self.move_time_max - self.move_time_min) / (1. - self.move_eps) * (1. - ratio)


    def send_right_arm_goal(self, positions):
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
                          "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = list(positions)
        jtp.velocities = [0.0] * len(positions)
        jtp.time_from_start = rospy.Trospy.logerrime(self.calc_move_time(self.last_right_buttons.axes[0]))
        jt.points.append(jtp)
        print("Goal: ")
        print(jt)
        self.right_command.publish(jt)

    def send_left_arm_goal(self, positions):
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                          "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = list(positions)
        jtp.velocities = [0.0] * len(positions)
        jtp.time_from_start = rospy.Time(self.calc_move_time(self.last_left_buttons.axes[0]))
        jt.points.append(jtp)
        self.left_command.publish(jt)


    def right_hand_run_ik_step(self):
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

        rospy.loginfo("Got pose: " + str(ps))
        sol = None
        retries = 0
        while not sol and retries < 10:
            sol = self.ik_right.get_ik(self.qinit_right,
                                       x, y, z,
                                       rx, ry, rz, rw,
                                       self.bx, self.by, self.bz,
                                       self.brx, self.bry, self.brz)
            retries += 1
        if sol:
            print "Solution found: (" + str(retries) + " retries)"
            print sol

            self.send_right_arm_goal(sol)
            self.qinit_right = sol
        else:
            print "NO SOLUTION FOUND for RIGHT :("

    def left_hand_run_ik_step(self):
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

        x = self.last_left_pose.pose.position.x
        y = self.last_left_pose.pose.position.y
        z = self.last_left_pose.pose.position.z

        rx = self.last_left_pose.pose.orientation.x
        ry = self.last_left_pose.pose.orientation.y
        rz = self.last_left_pose.pose.orientation.z
        rw = self.last_left_pose.pose.orientation.w

        rospy.loginfo("Got pose: " + str(ps))
        sol = None
        retries = 0
        while not sol and retries < 10:
            sol = self.ik_left.get_ik(self.qinit_left,
                                       x, y, z,
                                       rx, ry, rz, rw,
                                       self.bx, self.by, self.bz,
                                       self.brx, self.bry, self.brz)
            retries += 1
        if sol:
            print "Solution found: (" + str(retries) + " retries)"
            print sol

            self.send_left_arm_goal(sol)
            self.qinit_left = sol
        else:
            print "NO SOLUTION FOUND for LEFT :("

    def run_with_ik(self, arms='both', rate=20):
        self.qinit_right = [0., 0., 0., 0., 0., 0., 0.] 
        self.qinit_left = [0., 0., 0., 0., 0., 0., 0.]
        # x = y = z = 0.0
        # rx = ry = rz = 0.0
        # rw = 1.0
        self.bx = self.by = self.bz = 0.01
        self.brx = self.bry = self.brz = 0.5

        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            if arms == 'right' or arms == 'both':
                self.right_hand_run_ik_step()
            if arms == 'left' or arms == 'both':
                self.left_hand_run_ik_step()

            r.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--arms', choices=['left', 'right', 'both'], default='both', help='Which arms to use for teleop')
    parser.add_argument('--rate', default=20, type=int, help='what is the rate for IK')
    parser.add_argument('--filter', dest='filter', action='store_true', default=True, help='should filter collision states?')
    parser.add_argument('--no-filter', dest='filter', action='store_false')
    parser.add_argument('--button-start-script', '-a', type=str, help='The start script executed when grip button is pressed.')
    parser.add_argument('--button-stop-script',  '-z', type=str, help='The stop script executed when grip button is pressed.')
    args = parser.parse_args()

    print('Starting arguments: ', args)

    rospy.init_node('rad_teleop_pr2')
    nv = PR2Teleop(should_filter=args.filter, 
                   button_start_script=args.button_start_script, 
                   button_stop_script=args.button_stop_script)
    nv.run_with_ik(arms=args.arms, rate=args.rate)
