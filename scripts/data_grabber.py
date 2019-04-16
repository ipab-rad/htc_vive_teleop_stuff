import cv2
import roslib
import sensor_msgs
import sys
import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
import tf
import numpy as np
from os import makedirs
from os.path import join
from datetime import datetime
import message_filters

class DataGrabber:


    def __init__(self):

        self.bridge = CvBridge()

        self.recording = False
        self.recordingFolder = None
        rospy.Subscriber("/toggle_recording", Empty, self.toggle_recording)

        im_sub = message_filters.Subscriber("/kinect2/sd/image_color_rect",Image)
        joints_sub = message_filters.Subscriber("joint_states",JointState)
        # message_filters.Subscriber("/kinect2/sd/image_depth_rect",Image)

        synched_sub = message_filters.ApproximateTimeSynchronizer([im_sub, joints_sub], queue_size=25, slop=0.1)
        synched_sub.registerCallback(self.demo_callback)


    
    def toggle_recording(self, data):
        if self.recording:
            print("Turning off recording")
            self.recording = False
            self.recordingFolder = None
            # TODO: Should we scp the information off somewhere? 
            # TODO: Delete previous recordings from robot to save space?
        else:
            print("Turning on recording")
            self.recording = True
            self.recordingFolder = "Demos/Demo_{}".format(datetime.now()) 
            makedirs(self.recordingFolder)


    def demo_callback(self, im, joint_state):
        if not self.recording:
            return

        try:
            print("Recording demo data")

            cv_image = self.bridge.imgmsg_to_cv2(im, "bgr8")

            t_stamp = im.header.stamp.to_sec()
            im_path = join(self.recordingFolder, "kinect_colour_{}.jpg".format(t_stamp))
            vel_path = join(self.recordingFolder, "joint_vel_{}.txt".format(t_stamp))
            pose_path = join(self.recordingFolder, "joint_pos_{}.txt".format(t_stamp))
            names_path = join(self.recordingFolder, "joint_names{}.txt".format(t_stamp))

            cv2.imwrite(im_path, cv_image)
            np.savetxt(vel_path, joint_state.vel)
            np.savetxt(pose_path, joint_state.position)
            np.savetxt(names_path, joint_state.name, fmt='%s')

            print ("Saving image at time {}".format(t_stamp))
        except CvBridgeError as e:
            print ("No transform available")

 
if __name__ == '__main__':
    print("Starting Recording Node")

    rospy.init_node('image_grabber', anonymous=True)
    ic = DataGrabber()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down Recording Node")

