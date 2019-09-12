#!/usr/bin/env python

import argparse
import threading, subprocess, time
from math import radians

import rospy
import actionlib
# TF stuff
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
# from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Empty

import trac_ik_python
from trac_ik_python.trac_ik import IK
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

import moveit_commander
import copy
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
                 relative_control=False):
        self.robot = moveit_commander.RobotCommander()
        self.group_left = moveit_commander.MoveGroupCommander('left_arm')
        self.group_right = moveit_commander.MoveGroupCommander('right_arm')

        self.ik_right = IK("torso_lift_link",
                           "r_wrist_roll_link", solve_type='Manipulation1')
                           #"r_gripper_tool_frame")
        self.ik_left = IK("torso_lift_link",
                          "l_wrist_roll_link", solve_type='Manipulation1')

        self.relative_control = relative_control
        self.br = tf.TransformBroadcaster()
        self.tf_listener = TransformListener()

	self.recording = False
	self.start_pose_left = None
	self.start_pose_right = None

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

        # Last controller poses
        self.controller_last_left_pose = None
        self.controller_left_pose = rospy.Subscriber('/left_controller_as_posestamped_offset',
                                          PoseStamped,
                                          self.controller_left_cb, queue_size=1)
        self.controller_last_right_pose = None
        self.controller_right_pose = rospy.Subscriber('/right_controller_as_posestamped_offset',
                                           PoseStamped,
                                           self.controller_right_cb, queue_size=1)

        if self.relative_control:
            # Last robot poses
            self.robot_last_left_pose = None
            self.robot_left_pose = rospy.Subscriber('/l_wrist_roll_link_as_posestamped',
                                              PoseStamped,
                                              self.robot_left_cb, queue_size=1)
            self.robot_last_right_pose = None
            self.robot_right_pose = rospy.Subscriber('/r_wrist_roll_link_as_posestamped',
                                               PoseStamped,
                                               self.robot_right_cb, queue_size=1)
        


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


        self.ik_publish_rate = rospy.Publisher('/rate_ik_publish',
                                             Empty,
                                             queue_size=1)
        # self.msg_time_from_start_left = 0.8 # was 0.4
        # self.msg_time_from_start_right = 0.8 right_pose# was 0.4

        self.move_eps = 0.1 # The arms are not going to move if the button is pressed less than this
        self.move_time_min = 1.0
        self.move_time_max = 8.0

        self.record_pub = rospy.Publisher("/toggle_recording", Empty, queue_size=1)
        # Add for button debouncing
        self.record_state_change_time = time.time()
        self.record_state_change_hist_time = 1


        self.last_notmove_controller_right_pose = None
        self.last_notmove_robot_right_pose = None
        self.last_notmove_controller_left_pose = None
        self.last_notmove_robot_left_pose = None

        self.vibrate_right(3)
        # rospy.sleep(2.0)
        # self.left_gripper.open()
        # self.left_gripperecord_state_change_timer.close()

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

    def controller_left_cb(self, msg):
        self.controller_last_left_pose = msg
        # rospy.loginfo('Got a new left pose')

    def controller_right_cb(self, msg):
        self.controller_last_right_pose = msg
        # rospy.loginfo('Got a new right pose')

    def robot_left_cb(self, msg):
        self.robot_last_left_pose = msg
        # rospy.loginfo('Got a new robot left pose')

    def robot_right_cb(self, msg):
        self.robot_last_right_pose = msg
        # rospy.loginfo('Got a new robot right pose')



    def left_button_cb(self, msg):
        # print('Button press msg: ', msg)
        if msg.buttons[2] == 1:
            if (msg.axes[1] > 0):
                self.left_gripper.close()
            else:
                self.left_gripper.open()
        if msg.buttons[3] == 1:
            self.vibrate_right(1)
        if msg.buttons[4] == 1 and self.last_left_buttons.buttons[4] == 0:
            if time.time() - self.record_state_change_time > self.record_state_change_hist_time:
		# return to goal if recording is now over, else set starting pose
		print('Toggling recording and returning to start')
		if self.recording:
		    self.group_right.clear_pose_targets()
                    self.group_left.clear_pose_targets()
                    self.group_right.set_pose_target(self.start_pose_right.pose)
                    self.group_left.set_pose_target(self.start_pose_left.pose)
                    psr = self.group_right.go(wait=True)
                    psl = self.group_left.go(wait=True)
                    if psr and psl:
                        rospy.loginfo('Successfully planned to start')
                    else:
                        rospy.logerr('Failed to return to start positions')
			print('Failed to move from ',self.controller_last_left_pose, 'to ',self.start_pose_left)
		else:
                    self.tf_listener.waitForTransform("/hmd", "/base_link",rospy.Time.now(), rospy.Duration(1.0));
		    self.start_pose_left = self.tf_listener.transformPose('base_link',self.controller_last_left_pose)
                    self.start_pose_right = self.tf_listener.transformPose('base_link',self.controller_last_right_pose)
                # update last time command was executed!
                self.record_state_change_time = time.time()

                self.record_pub.publish()
		self.recording = not self.recording
                self.vibrate_left(1, strength=0.6)

        self.last_left_buttons = msg


    def right_button_cb(self, msg):
        # print('Button press msg: ', msg)
        if msg.buttons[2] == 1:
            if (msg.axes[1] > 0):
                self.right_gripper.close()
            else:
                self.right_gripper.open()
        if msg.buttons[3] == 1:
            self.vibrate_right(1)
        if msg.buttons[4] == 1 and self.last_right_buttons.buttons[4] == 0:
            if time.time() - self.record_state_change_time > self.record_state_change_hist_time:
                # update last time command was executed!
                self.record_state_change_time = time.time()

                self.record_pub.publish()
                self.vibrate_right(1, strength=0.9)

        self.last_right_buttons = msg

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
        jtp.accelerations = [0.0] * len(positions)
        jtp.time_from_start = rospy.Time(self.calc_move_time(self.last_right_buttons.axes[0]))
        jt.points.append(jtp)
        print("Goal: ")
        #print(jt)
        self.right_command.publish(jt)

    def send_left_arm_goal(self, positions):
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                          "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = list(positions)
        jtp.velocities = [0.0] * len(positions)
        jtp.accelerations = [0.0] * len(positions)
        jtp.time_from_start = rospy.Time(self.calc_move_time(self.last_left_buttons.axes[0]))
        jt.points.append(jtp)
        self.left_command.publish(jt)


    def right_hand_run_ik_step(self):
        ps = self.controller_last_right_pose
        if ps is None:
            # r.sleep()
            rospy.logdebug("No last right pose...")
            return

        if self.last_right_buttons is None:
            rospy.logdebug('No button information received (right).')
            return

        if self.last_right_buttons.axes[0] < self.move_eps:
            rospy.logdebug('Right Button pressed too easy.')
            # self.no_move_trigger = True
            if self.relative_control:
                self.last_notmove_controller_right_pose = self.controller_last_right_pose
                self.last_notmove_robot_right_pose = self.robot_last_right_pose
                # rospy.loginfo(self.last_notmove_robot_right_pose)
            return 

        print('Sending a command move message!!!!!!!!')
        if self.relative_control:
            if (self.tf_listener.frameExists("last_notmoved_controller_right_pose") and 
                self.tf_listener.frameExists("last_notmoved_robot_right_pose")):
                # The transform is thejtp.accelerations = [0.0] * len(positions)re! Let's calculate offset
                p1 = PoseStamped()
                p1.header.frame_id = "right_controller_offset"
                p1.header.stamp = rospy.Time(0) #self.tf_listener.getLatestCommonTime("/right_controller_offset", "/last_notmoved_controller_right_pose")
                p1.pose.orientation.w = 1.0    # Neutral orientation
                # p1.pose.position.x = -0.18
                p_in_base = self.tf_listener.transformPose("last_notmoved_controller_right_pose", p1)
                # print(p_in_base)
                # return

                p1_robot = PoseStamped()
                p1_robot.header.frame_id = 'last_notmoved_robot_right_pose'
                p1_robot.header.stamp = rospy.Time(0) #self.tf_listener.getLatestCommonTime("/last_notmoved_robot_right_pose", "/torso_lift_link")
                p1_robot.pose.position = p_in_base.pose.position
                p1_robot.pose.orientation = p_in_base.pose.orientation
                p_in_robot = self.tf_listener.transformPose('torso_lift_link', p1_robot)
                
                print(p_in_robot)
                ps = p_in_robot

                self.br.sendTransform((ps.pose.position.x,
                                   ps.pose.position.y,
                                   ps.pose.position.z),
                     (ps.pose.orientation.x,
                      ps.pose.orientation.y,
                      ps.pose.orientation.z,
                      ps.pose.orientation.w),
                     rospy.Time.now(),
                     'target_r_wrist',
                     'torso_lift_link'
                     )

                # rospy.loginfo('Offset from last movemenent: ', p_in_base) # raises an error?
            else:
                rospy.warn('Transform `last_notmoved_robot_right_pose` or `last_notmoved_controller_right_pose` does not exist.')

        # return
        x = ps.pose.position.x
        y = ps.pose.position.y
        z = ps.pose.position.z - 0.5

        rx = ps.pose.orientation.x
        ry = ps.pose.orientation.y
        rz = ps.pose.orientation.z
        rw = ps.pose.orientation.w

        # rospy.loginfo("Got pose: " + str(ps))
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
        ps = self.controller_last_left_pose
        if ps is None:
            # r.sleep()
            rospy.logdebug("No last left pose...")
            return

        if self.last_left_buttons is None:
            rospy.logdebug('No button information received (left).')
            return

        if self.last_left_buttons.axes[0] < self.move_eps:
            # print('Left button pressed too easy.')
            return

        x = self.controller_last_left_pose.pose.position.x 
        y = self.controller_last_left_pose.pose.position.y
        z = self.controller_last_left_pose.pose.position.z - 0.5

        rx = self.controller_last_left_pose.pose.orientation.x
        ry = self.controller_last_left_pose.pose.orientation.y
        rz = self.controller_last_left_pose.pose.orientation.z
        rw = self.controller_last_left_pose.pose.orientation.w

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

    def republish_tf_frames(self):
        if self.last_notmove_robot_right_pose:
            self.br.sendTransform((self.last_notmove_robot_right_pose.pose.position.x,
                                   self.last_notmove_robot_right_pose.pose.position.y,
                                   self.last_notmove_robot_right_pose.pose.position.z),
                     (self.last_notmove_robot_right_pose.pose.orientation.x,
                      self.last_notmove_robot_right_pose.pose.orientation.y,
                      self.last_notmove_robot_right_pose.pose.orientation.z,
                      self.last_notmove_robot_right_pose.pose.orientation.w),
                     rospy.Time.now(),
                     'last_notmoved_robot_right_pose',
                     self.last_notmove_robot_right_pose.header.frame_id)
        if self.last_notmove_controller_right_pose:
            self.br.sendTransform((self.last_notmove_controller_right_pose.pose.position.x,
                                   self.last_notmove_controller_right_pose.pose.position.y,
                                   self.last_notmove_controller_right_pose.pose.position.z),
                     (self.last_notmove_controller_right_pose.pose.orientation.x,
                      self.last_notmove_controller_right_pose.pose.orientation.y,
                      self.last_notmove_controller_right_pose.pose.orientation.z,
                      self.last_notmove_controller_right_pose.pose.orientation.w),
                     rospy.Time.now(),
                     'last_notmoved_controller_right_pose',
                     self.last_notmove_controller_right_pose.header.frame_id)
        if self.last_notmove_robot_left_pose:
            self.br.sendTransform((self.last_notmove_robot_left_pose.pose.position.x,
                                   self.last_notmove_robot_left_pose.pose.position.y,
                                   self.last_notmove_robot_left_pose.pose.position.z),
                     (self.last_notmove_robot_left_pose.pose.orientation.x,
                      self.last_notmove_robot_left_pose.pose.orientation.y,
                      self.last_notmove_robot_left_pose.pose.orientation.z,
                      self.last_notmove_robot_left_pose.pose.orientation.w),
                     rospy.Time.now(),
                     'last_notmoved_robot_left_pose',
                     self.last_notmove_robot_left_pose.header.frame_id)
        if self.last_notmove_controller_left_pose:
            self.br.sendTransform((self.last_notmove_controller_left_pose.pose.position.x,
                                   self.last_notmove_controller_left_pose.pose.position.y,
                                   self.last_notmove_controller_left_pose.pose.position.z),
                     (self.last_notmove_controller_left_pose.pose.orientation.x,
                      self.last_notmove_controller_left_pose.pose.orientation.y,
                      self.last_notmove_controller_left_pose.pose.orientation.z,
                      self.last_notmove_controller_left_pose.pose.orientation.w),
                     rospy.Time.now(),
                     'last_notmoved_controller_left_pose',
                     self.last_notmove_controller_left_pose.header.frame_id)

    def run_with_ik(self, rate, arms='both'):
        self.qinit_right = [0., 0., 0., 0., 0., 0., 0.] 
        self.qinit_left = [0., 0., 0., 0., 0., 0., 0.]
        # x = y = z = 0.0
        # rx = ry = rz = 0.0
        # rw = 1.0
        self.bx = self.by = self.bz = 1e-7#0.01
        self.brx = self.bry = self.brz = 1e-7 #0.5

        r = rospy.Rate(rate)
        while not rospy.is_shutdown():

            self.ik_publish_rate.publish()
            
            if self.relative_control:
                self.republish_tf_frames()

            if arms == 'right' or arms == 'both':
                self.right_hand_run_ik_step()
            if arms == 'left' or arms == 'both':
                self.left_hand_run_ik_step()


            r.sleep()


if __name__ == '__main__':
    print("Location of trak_ik: ", trac_ik_python.__file__)
    parser = argparse.ArgumentParser()
    parser.add_argument('--arms', choices=['left', 'right', 'both'], default='both', help='Which arms to use for teleop')
    parser.add_argument('--rate', default=30, type=int, help='what is the rate for IK')
    parser.add_argument('--filter', dest='filter', action='store_true', default=True, help='should filter collision states?')
    parser.add_argument('--relative-control', dest='relative_control', action='store_true', default=False, help='Relative control.')
    parser.add_argument('--no-filter', dest='filter', action='store_false')
    parser.add_argument('--button-start-script', '-a', type=str, help='The start script executed when grip button is pressed.')
    parser.add_argument('--button-stop-script',  '-z', type=str, help='The stop script executed when grip button is pressed.')
    args, unknown_args = parser.parse_known_args()

    print('Starting arguments: ', args)

    rospy.init_node('rad_teleop_pr2')
    nv = PR2Teleop(should_filter=args.filter, 
                   relative_control=args.relative_control)
    nv.run_with_ik(arms=args.arms, rate=args.rate)
