#! /usr/bin/env python
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
import os
from os import makedirs
from os.path import join
from datetime import datetime
import message_filters
import argparse

class DataGrabber:

    def __init__(self, image_topics, root_path):

        self.bridge = CvBridge()

        self.recording = False
        self.recordingFolder = None
        self.root_path = root_path
        self.image_topics = image_topics

        rospy.loginfo(['Image topics for recording: ', image_topics])

        rospy.Subscriber("/toggle_recording", Empty, self.toggle_recording)

        im_subs = [message_filters.Subscriber(image_topic, Image) for image_topic in self.image_topics]
        joints_sub = message_filters.Subscriber("joint_states", JointState)
        # message_filters.Subscriber("/kinect2/sd/image_depth_rect",Image)

        synched_sub = message_filters.ApproximateTimeSynchronizer(im_subs + [joints_sub], queue_size=250, slop=0.05)
        synched_sub.registerCallback(self.demo_callback)

        self.save_rate_publish = rospy.Publisher('/rate_save_data_grabber',
                                             Empty, queue_size=1)

    
    def toggle_recording(self, data):
        if self.recording:
            print("Turning off recording for {}".format(self.recordingFolder))
            self.recording = False
            self.recordingFolder = None
            # TODO: Should we scp the information off somewhere? 
            # TODO: Delete previous recordings from robot to save space?
        else:
            print("Turning on recording")
            self.recording = True
            self.recordingFolder = os.path.join(self.root_path, 
                                "Demos/Demo_{}".format(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')))
            makedirs(self.recordingFolder)
            # print("Current Directory:", os.getcwd())
            print("Folder made at {}".format(self.recordingFolder))


    def demo_callback(self, *data):
        if not self.recording:
            return

        try:
            rospy.logdebug('RECORDING NEW DATA!!!')
            # Split datarecording
            joint_state = data[-1]
            ims = data[:-1]
            
            # Get timestamp
            im = ims[0]
            t_stamp = int(im.header.stamp.to_nsec())
            out_folder = self.recordingFolder # Make sure to safe folder, as it's async and can be removed mid execution.

            print(self.image_topics, len(data), len(ims), len(self.image_topics))
            # Save images
            for im, image_topic in zip(ims, self.image_topics):
                if 'depth' not in image_topic:
                    cv_image = self.bridge.imgmsg_to_cv2(im, "bgr8")
                    image_name = image_topic.strip('/').replace('/', '_') + '_{}.jpg'
                    im_path = join(out_folder, image_name.format(t_stamp))
                    cv2.imwrite(im_path, cv_image)
                else:
                    rospy.logdebug('this is a depth image.')
                    # print(im)
                    cv_image = self.bridge.imgmsg_to_cv2(im, "passthrough")/10000. * 255.
                    print('Min ', np.min(cv_image), 'max: ', np.max(cv_image))
                    image_name = image_topic.strip('/').replace('/', '_') + '_{}.jpg'
                    im_path = join(out_folder, image_name.format(t_stamp))
                    cv2.imwrite(im_path, cv_image)
            
            # Save joint states
            vel_path = join(out_folder, "joint_vel_{}.txt".format(t_stamp))
            pose_path = join(out_folder, "joint_pos_{}.txt".format(t_stamp))
            names_path = join(out_folder, "joint_names_{}.txt".format(t_stamp))
            effort_path = join(out_folder, "joint_effort_{}.txt".format(t_stamp))

            np.savetxt(vel_path, joint_state.velocity)
            np.savetxt(pose_path, joint_state.position)
            np.savetxt(pose_path, joint_state.effort)
            np.savetxt(names_path, joint_state.name, fmt='%s')

            self.save_rate_publish.publish()
            print ("Saving image at time {}".format(t_stamp))
        except CvBridgeError as e:
            print ("No transform available")

 
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--imtopic", action='append', help="The ROS image topic path which the grabber will record") # e.g.  default=["/kinect2/sd/image_color_rect"], 
    parser.add_argument("--root_path", default="/home/michael/vive_ws/src/htc_vive_teleop_stuff/scripts", help="The root folder where to save the demo files.")
    args, unknown_args = parser.parse_known_args()

    rospy.init_node('image_grabber', anonymous=True)
    ic = DataGrabber(args.imtopic, args.root_path)
    print("Recording Node Online")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down Recording Node")

