import sys, time, collections
import copy, math
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import argparse

from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
import actionlib

def wait_for_state_update(box_name="table", box_is_known=False, box_is_attached=False, timeout=4):
   
   start = rospy.get_time()
   seconds = rospy.get_time()
   while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
         return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

   # If we exited the while loop without returning then we timed out
   return False

def add_collision_table(box_name="table", timeout=4):
   rospy.sleep(0.1)

   table_height = 0.025

   p = geometry_msgs.msg.PoseStamped()
   p.header.frame_id = robot.get_planning_frame()
   p.pose.position.x = 0.6 # fwd
   p.pose.position.y = 0 # left/right
   p.pose.position.z = 0.73 - table_height/2.
   # p.pose.orientation.w = 0.924
   # p.pose.orientation.z = 0.383
   p.pose.orientation.w = 1.
   scene.add_box(box_name, p, (0.8, 1.5, table_height))
   
   return wait_for_state_update(box_name=box_name, box_is_known=True)

def remove_collision_table(box_name="table", timeout=4):

   scene.remove_world_object(box_name)
   return wait_for_state_update(box_name=box_name, box_is_known=False)


if __name__ == "__main__":

   parser = argparse.ArgumentParser(description='Add/remove objects from the planing scene.')
   parser.add_argument('--add', action='store_true', default=False, help='Render the env')

   rospy.init_node('config_scene',
                anonymous=True)

   robot = moveit_commander.RobotCommander()
   scene = moveit_commander.PlanningSceneInterface()

   rospy.sleep(1)

   success = add_collision_table(box_name="table")
   # success = remove_collision_table(box_name="table")
   
   print("Add table success:", success)
   
