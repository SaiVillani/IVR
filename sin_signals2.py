#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class sinSignal:
    def __init__(self):
        # initialize the node named sin_signal2
        rospy.init_node('sin_signal2', anonymous=True)

        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback)

        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback)

        # Get time
        self.time = rospy.get_time()

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def callback(self, data):

        # Make the rotations
        # Get time
        self.time_now = rospy.get_time()
        secs = self.time_now - self.time
        # self.time = self.time_now
        # print(secs)
        self.joint1 = Float64()
        self.joint1.data = np.pi * np.sin(secs * np.pi / 28)
        self.joint3 = Float64()
        self.joint3.data = (np.pi / 2) * np.sin(secs * np.pi / 20)
        self.joint4 = Float64()
        self.joint4.data = (np.pi / 2) * np.sin(secs * np.pi / 18)

        try:
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)

# call the class
def main(args):
    ic = sinSignal()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
