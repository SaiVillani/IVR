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
import os
import random

class helper:
    def __init__(self):
        # initialize the node named sin_signal2
        rospy.init_node('helper', anonymous=True)

        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        # self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)

        # initialize a publisher to send images from camera2 to a topic named image_topic2
        # self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

        # Get time
        self.time = rospy.get_time()

        # Count
        self.counter = 0

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def angle_to_pos(self, joint_angles):
        # Publish the results
        try:
            self.robot_joint1_pub.publish(joint_angles[0])
            cv2.waitKey(4500)
            self.robot_joint3_pub.publish(joint_angles[1])
            cv2.waitKey(4500)
            self.robot_joint4_pub.publish(joint_angles[2])
            cv2.waitKey(4500)
        except CvBridgeError as e:
            print(e)
        # cv2.waitKey(5)
        return self.get_end_effector_pos(self.cv_image1, self.cv_image2)

    def detect_red(self, image):
        # Isolate the blue colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_green(image)
        circle2Pos = self.detect_yellow(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 4 / np.sqrt(dist)

    def get_end_effector_pos(self, image1, image2):
        # Get the yz coordinates from camera 1
        a = self.pixel2meter(image1)
        center_yz = a * self.detect_green(image1)
        circle1Pos_yz = a * self.detect_yellow(image1)

        try:
            circle2Pos_yz = a * self.detect_blue(image1)
        except:
            circle2Pos_yz = circle1Pos_yz

        # Cases where red circle is not visible is when
        # it is behind the blue circle (or if the blue circle is also behind the yellow) usually
        # This is a simplifying assumption
        try:
            circle3Pos_yz = a * self.detect_red(image1)
        except:
            circle3Pos_yz = circle2Pos_yz

        # Get the xz coordinates from camera 2
        b = self.pixel2meter(image2)
        center_xz = b * self.detect_green(image2)
        circle1Pos_xz = b * self.detect_yellow(image2)
        try:
            circle2Pos_xz = b * self.detect_blue(image2)
        except:
            circle2Pos_xz = circle1Pos_xz
        try:
            circle3Pos_xz = b * self.detect_red(image2)
        except:
            circle3Pos_xz = circle1Pos_xz

        return np.array([circle3Pos_xz[0], circle3Pos_yz[0], (circle3Pos_xz[1] + circle3Pos_yz[1]) / 2])

    # Recieve data, process it, and publish
    def callback1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def callback2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        values1 = [2.1, 0.1, 0.1]
        values2 = [1.1, 0.7, 1.1]
        values3 = [-1.1, -0.1, -0.1]
        values4 = [-2.7, -0.1, -1.0]
        values5 = [0.7, 0.4, 1.2]
        values6 = [-2.7, 0.4, 1.3]
        values7 = [-0.6, 1.4, 0.6]
        values8 = [1.6, -1.4, -0.6]
        values9 = [-0.9, 1.1, -0.3]
        values10 = [2.4, 0.5, 1.1]

        values_lst = [values1, values2,
                    values3,
                    values4,
                    values5,
                    values6,
                    values7,
                    values8,
                    values9,
                    values10]

        if self.counter < 10:
            print(self.counter + 1)
            results = self.forward_kinematics_test(values_lst[self.counter])
            actual = self.angle_to_pos(values_lst[self.counter])
            cv2.waitKey(4000)
            print("real values:", values_lst[self.counter])
            print("real position:", actual)
            print("FW: ", results)


        self.counter += 1


        # results1 = self.forward_kinematics_test(values1)
        # results2 = self.forward_kinematics_test(values2)
        # results3 = self.forward_kinematics_test(values3)
        # results4 = self.forward_kinematics_test(values4)
        # results5 = self.forward_kinematics_test(values5)
        # results6 = self.forward_kinematics_test(values6)
        # results7 = self.forward_kinematics_test(values7)
        # results8 = self.forward_kinematics_test(values8)
        # results9 = self.forward_kinematics_test(values9)
        # results10 = self.forward_kinematics_test(values10)

        # actual1 = self.angle_to_pos(values1)
        # actual2 = self.angle_to_pos(values2)
        # actual3 = self.angle_to_pos(values3)
        # actual4 = self.angle_to_pos(values4)
        # actual5 = self.angle_to_pos(values5)
        # actual6 = self.angle_to_pos(values6)
        # actual7 = self.angle_to_pos(values7)
        # actual8 = self.angle_to_pos(values8)
        # actual9 = self.angle_to_pos(values9)
        # actual10 = self.angle_to_pos(values10)

        # print("real values:", values1)
        # print("real values:", values2)
        # print("real values:", values3)
        # print("real values:", values4)
        # print("real values:", values5)
        # print("real values:", values6)
        # print("real values:", values7)
        # print("real values:", values8)
        # print("real values:", values9)
        # print("real values:", values10)
        #
        # print("real position:", actual1)
        # print("real position:", actual2)
        # print("real position:", actual3)
        # print("real position:", actual4)
        # print("real position:", actual5)
        # print("real position:", actual6)
        # print("real position:", actual7)
        # print("real position:", actual8)
        # print("real position:", actual9)
        # print("real position:", actual10)
        #
        # print("FW: ", results1)
        # print("FW: ", results2)
        # print("FW: ", results3)
        # print("FW: ", results4)
        # print("FW: ", results5)
        # print("FW: ", results6)
        # print("FW: ", results7)
        # print("FW: ", results8)
        # print("FW: ", results9)
        # print("FW: ", results10)


    def forward_kinematics_test(self,value):
        angles = value
        end_effector = np.array([2.8 * np.cos(angles[0]) * np.sin(angles[2]) + np.sin(angles[0]) * np.sin(
          angles[1]) * (2.8 * np.cos(angles[2]) + 3.2),
                               2.8 * np.sin(angles[0]) * np.sin(angles[2]) - np.cos(angles[0]) * np.sin(
                                   angles[1]) * (2.8 * np.cos(angles[2]) + 3.2),
                               np.cos(angles[1]) * (2.8 * np.cos(angles[2]) + 3.2) + 4.0])
        return end_effector

# call the class
def main(args):
    ic = helper()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
