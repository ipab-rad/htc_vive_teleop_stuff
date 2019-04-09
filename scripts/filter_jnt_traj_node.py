#!/usr/bin/env python

import argparse
import rospy
# TF stuff
# from geometry_msgs.msg import PoseStamped
# from tf.transformations import euler_from_quaternion
# from math import radians
from trajectory_msgs.msg import JointTrajectory
# from sensor_msgs.msg import Joy
# from std_msgs.msg import Float64

# from trac_ik_python.trac_ik import IK

# import threading

# # from trajectory_msgs.msg import *
# # from actionlib_msgs.msg import *
# from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
# # from sensor_msgs.msg import JointState
# import actionlib


import query_state_validity_service_call as state_validity


class JointTrajFilter(object):
    """docstring for JointTrajFilter"""
    def __init__(self, arm):
        super(JointTrajFilter, self).__init__()
        self.arm = arm

        self.command_pub = rospy.Publisher('/' + self.arm[0] + '_arm_controller/command',
                                            JointTrajectory,
                                            queue_size=1)

        self.command_unfiltered_sub = rospy.Subscriber('/' + self.arm[0] + '_arm_controller/command_filter',
                                          JointTrajectory,
                                          self.unfiltered_command_cb, queue_size=1)
        self.state_service = state_validity.get_check_state_service(persistent=True)
        print('Started: ', self.arm)

    def unfiltered_command_cb(self, msg):
        self.unfiltered_command_cb = msg
        print('msg:', msg)
        valid = state_validity.check_state(names=msg.joint_names,
                                           positions=msg.points[-1].positions,
                                           service=self.state_service)
        print(valid)
        if valid:
            self.command_pub.publish(msg)


def main():
    rospy.init_node('joint_state_filter')
    left = JointTrajFilter(arm = 'left')
    right = JointTrajFilter(arm = 'right')
    rospy.spin()

if __name__ == '__main__':
    main()