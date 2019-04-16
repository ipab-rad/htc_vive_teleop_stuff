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

class DataGrabber:


    def __init__(self):

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.image_sub = rospy.Subscriber("/kinect2/sd/image_color_rect",Image,self.im_callback,queue_size=1)
        self.joint_sub = rospy.Subscriber("joint_states",JointState,self.j_callback,queue_size=1)
        self.recording_sub = rospy.Subscriber("/toggle_recording", Empty, self.toggle_recording)
        #self.depth_sub = rospy.Subscriber("/kinect2/sd/image_depth_rect",Image,self.callback_depth)

        self.vel = []
        self.pos = []
        self.names = []

        self.recording = False
        self.recordingFolder = None

    
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


    def im_callback(self,data):
        if not self.recording:
            return

        try:
            print("Recording demo data")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            t_stamp = data.header.stamp.to_sec()
            im_path = join(self.recordingFolder, "kinect_colour_{}.jpg".format(t_stamp))
            vel_path = join(self.recordingFolder, "joint_vel_{}.txt".format(t_stamp))
            pose_path = join(self.recordingFolder, "joint_pos_{}.txt".format(t_stamp))
            names_path = join(self.recordingFolder, "joint_names{}.txt".format(t_stamp))

            cv2.imwrite(im_path, cv_image)
            np.savetxt(vel_path, self.vel)
            np.savetxt(pose_path, self.pos)
            np.savetxt(names_path, self.names, fmt='%s')

            print ("Saving image at time {}".format(t_stamp))
        except CvBridgeError as e:
            print ("No transform available")
             
    def j_callback(self,data):
        try:
            self.vel = data.velocity
            self.pos = data.position
            self.names = data.name

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

