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

class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)

        # initialize publishers to send joints' angular position to topics
        self.joints_pub2_ja1 = rospy.Publisher("joint_angle_1", Float64, queue_size=10)
        self.joints_pub2_ja3 = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joints_pub2_ja4 = rospy.Publisher("joint_angle_4", Float64, queue_size=10)

        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

        # List to store last ten joint angles
        self.last_ten = []

        # Flag to check if the circles are detected successfully
        self.red_flag1 = False
        self.red_flag2 = False
        self.blue_flag1 = False
        self.blue_flag2 = False

    # Recieve data from camera 1, process it
    def callback1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # loading template for links as binary image
        self.link1 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link1.png', 1), (200, 200, 200), (255, 255, 255))
        self.link2 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link2.png', 1), (200, 200, 200), (255, 255, 255))
        self.link3 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link3.png', 1), (200, 200, 200), (255, 255, 255))

        # Get joint angles from camera 1
        # print('camera1')
        # self.joints_est1 = self.detect_joint_angles1(self.cv_image1)

        # # Chamfer
        # self.joints_est1_chamfer = self.detect_joint_angles_chamfer1(self.cv_image1)

        # Blue not detected by camera 2
        # or Red not detected by camera 2
        # if (self.blue_flag2 == True) | (self.red_flag2 == True):
        #     self.joints_est1[1] = -np.pi/2
        #     self.joints_est1_chamfer[1] = -np.pi / 2
        #
        # # Case where camera 1 fails to detect blue ball
        # if self.blue_flag1 == True:
        #     self.joints_est1[1] = 0
        #     self.joints_est1_chamfer[1] = 0

        # # Publish the results
        # try:
        #     self.joints_pub_ja3.publish(np.sign(self.joints_est1[1]) * min(np.pi / 2, abs(self.joints_est1[1])))
        #     self.joints_pub2_ja3.publish(np.sign(self.joints_est1_chamfer[1]) * min(np.pi / 2, abs(self.joints_est1_chamfer[1])))
        # except CvBridgeError as e:
        #     print(e)

    # Recieve data from camera 2, process it
    def callback2(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # loading template for links as binary image
        self.link1 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link1.png', 1), (200, 200, 200), (255, 255, 255))
        self.link2 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link2.png', 1), (200, 200, 200), (255, 255, 255))
        self.link3 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link3.png', 1), (200, 200, 200), (255, 255, 255))

        # # Get joint angles from camera 2
        # self.joints_est2 = self.detect_joint_angles2(self.cv_image2)

        # # Chamfer
        # self.joints_est2_chamfer = self.detect_joint_angles_chamfer2(self.cv_image2)
        #
        # print('Cam1', self.joints_est1_chamfer)
        # print('Cam2', self.joints_est2_chamfer)
        # print('Blue flag 1', self.blue_flag1)
        # print('Blue flag 2',self.blue_flag2)

        # Cases where camera 2 fails to detect red ball, switch camera
        # if self.red_flag2 == True:
        #     self.joints_est2[2] = self.joints_est1[2]
        #     self.joints_est2_chamfer[2] = self.joints_est1_chamfer[2]

        self.joint_angle_estimations = self.angle_estimation(self.cv_image1, self.cv_image2)

        # Publish the results
        try:
            self.joints_pub2_ja1.publish(self.joint_angle_estimations[0])
            self.joints_pub2_ja3.publish(self.joint_angle_estimations[1])
            self.joints_pub2_ja4.publish(self.joint_angle_estimations[2])
        except CvBridgeError as e:
            print(e)

    def angle_estimation(self, image1, image2):

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

        # Start with joint angle 1 detection
        # Use the x-y coordinate
        center_xy = np.array([center_xz[0],center_yz[0]])
        circle1Pos_xy = np.array([circle1Pos_xz[0], circle1Pos_yz[0]])
        circle2Pos_xy = np.array([circle2Pos_xz[0], circle2Pos_yz[0]])
        circle3Pos_xy = np.array([circle3Pos_xz[0], circle3Pos_yz[0]])

        # Blue - yellow and get angle
        diff_vec = circle2Pos_xy - circle1Pos_xy
        ja1_raw = np.arctan2(np.abs(diff_vec[0]), np.abs(diff_vec[1]))

        # If blue circle is left side (xy coordinate) from camera 1 angle
        # Flipped side of usual coordinates
        if diff_vec[1] <= 0:
            # Top side
            if diff_vec[0] <= 0:
                ja1 = np.pi - ja1_raw
                # print(ja1)
            # Bottom
            else:
                ja1 = -(np.pi - ja1_raw)
                # print(ja1)
        # Right
        else:
            # Top side
            if diff_vec[0] <= 0:
                ja1 = ja1_raw
                # print(ja1)
            # Right side
            else:
                ja1 = - ja1_raw
                # print(ja1)

        # Joint angle 3 estimation

        # Check to see if the length of the link 3 is within the threshold
        thres = 0.5

        # Compute distance using camera1
        dist1 = np.sqrt(np.sum((circle1Pos_yz - circle2Pos_yz) ** 2))

        # Compute distance using camera2
        dist2 = np.sqrt(np.sum((circle1Pos_xz - circle2Pos_xz) ** 2))

        # Camera 1 as priority
        if np.abs(dist1 - 3.2) <= thres:
            ja3 = np.arctan2(np.abs(circle1Pos_yz[0] - circle2Pos_yz[0]), np.abs(circle1Pos_yz[1] - circle2Pos_yz[1]))
        # Try camera 2
        elif np.abs(dist2 - 3.2) <= thres:
            ja3 = np.arctan2(np.abs(circle1Pos_xz[0] - circle2Pos_xz[0]), np.abs(circle1Pos_xz[1] - circle2Pos_xz[1]))
        # If both camera fails
        # Get the closer of the two
        else:
            if np.abs(dist1 - 3.2) <= np.abs(dist2 - 3.2):
                ja3 = np.arctan2(np.abs(circle1Pos_yz[0] - circle2Pos_yz[0]),
                                 np.abs(circle1Pos_yz[1] - circle2Pos_yz[1]))
            else:
                ja3 = np.arctan2(np.abs(circle1Pos_xz[0] - circle2Pos_xz[0]),
                                 np.abs(circle1Pos_xz[1] - circle2Pos_xz[1]))

        # Check to see if the image is behind or in front relative to yellow circle for camera 1
        # This corresponds to left and right of the camera 2 image
        # and behind => different signs for joint angles 1 and 3; in front => same sign
        # Left side
        weight = 0.85
        if circle1Pos_xz[0] - circle2Pos_xz[0] >= 0:
            if len(self.last_ten) > 0:
                total_weights = 0
                w_arr = np.zeros(3)
                diff_trend = np.zeros(3)
                rev_lst = self.last_ten[::-1]
                for i, arr in enumerate(rev_lst):
                    w_arr += (weight**i) * arr
                    total_weights += weight**i
                    if len(rev_lst) >= 2:
                        if i + 1 < len(rev_lst):
                            diff_trend += arr - rev_lst[i+1]

                # Normalise the weighted array
                w_arr = w_arr/total_weights

                if (diff_trend[0] != 0) | (diff_trend[1] != 0):
                    diff_trend = diff_trend/(len(rev_lst) - 1)
                ja1_forecast = rev_lst[0][0] + diff_trend[0]
                ja3_forecast = rev_lst[0][1] + diff_trend[1]

                ja1_forecast = max(min((ja1_forecast) , np.pi), -np.pi)
                ja3_forecast = max(min((ja3_forecast) , np.pi), -np.pi)

                ja1_thres =0.35
                ja3_thres = 0.35
                if np.abs(w_arr[0] - ja1) <= ja1_thres:
                    # Adjust the ja3 accordingly
                    ja3 = -np.sign(ja1) * np.abs(ja3)
                else:
                    # Adjust ja1
                    ja1_adj1 = min(np.pi + ja1, np.pi)
                    ja1_adj2 = max(- np.pi + ja1, -np.pi)

                    if np.abs(w_arr[0] - ja1_adj1) <= np.abs(w_arr[0] - ja1_adj2):
                        ja1 = ja1_adj1
                    else:
                        ja1 = ja1_adj2
                    # Prevent sudden spikes
                    if np.abs(w_arr[0] - ja1) >= ja1_thres:
                        # ja1 = ja1_forecast
                        ja1= max(min((ja1 + ja1_forecast)/2, np.pi),-np.pi)
                        # print(ja1)
                        # print(ja1_forecast)
                # Prevent sudden spikes
                if np.abs(w_arr[1] - ja3) >= ja3_thres:
                    # ja3 = ja3_forecast
                    ja3 = max(min((ja3 + ja3_forecast)/2, np.pi),-np.pi)

            else:
                ja3 = -np.sign(ja1) * np.abs(ja3) # Adjust ja3
        # Right side of camera 2
        else:
            if len(self.last_ten) > 0:
                total_weights = 0
                w_arr = np.zeros(3)
                diff_trend = np.zeros(3)
                rev_lst = self.last_ten[::-1]
                for i, arr in enumerate(rev_lst):
                    w_arr += (weight**i) * arr
                    total_weights += weight**i
                    if len(rev_lst) >= 2:
                        if i + 1 < len(rev_lst):
                            diff_trend += arr - rev_lst[i+1]
                # Normalise the weighted array
                w_arr = w_arr/total_weights

                if (diff_trend[0] != 0) | (diff_trend[1] != 0):
                    diff_trend = diff_trend/(len(rev_lst) - 1)
                ja1_forecast = rev_lst[0][0] + diff_trend[0]
                ja3_forecast = rev_lst[0][1] + diff_trend[1]

                ja1_forecast = max(min((ja1_forecast) , np.pi), -np.pi)
                ja3_forecast = max(min((ja3_forecast) , np.pi), -np.pi)

                ja1_thres =0.35
                ja3_thres = 0.35
                if np.abs(w_arr[0] - ja1) <= ja1_thres:
                    # Adjust the ja3 accordingly
                    ja3 = np.sign(ja1) * np.abs(ja3)
                else:
                    # Adjust ja1
                    ja1_adj1 = np.pi - np.abs(ja1)
                    ja1_adj2 = - np.pi + np.abs(ja1)

                    if np.abs(w_arr[0] - ja1_adj1) <= np.abs(w_arr[0] - ja1_adj2):
                        ja1 = ja1_adj1
                    else:
                        ja1 = ja1_adj2
                    # Prevent sudden spikes
                    if np.abs(w_arr[0] - ja1) >= ja1_thres:
                        ja1= max(min((ja1 + ja1_forecast)/2, np.pi),-np.pi)
                        # ja1 = ja1_forecast
                        print(ja1)
                        # print(ja1_forecast)
                # Prevent sudden spikes
                if np.abs(w_arr[1] - ja3) >= ja3_thres:
                    # ja3 = max(min((ja3 + ja3_forecast)/2, np.pi),-np.pi)
                    ja3 = ja3_forecast

            else:
                ja3 = np.sign(ja1) * np.abs(ja3) # Adjust ja3

        # Estimation of joint angle 4
        # # Camera 2 as priority
        # if np.abs(dist2 - 3.2) <= thres:
        #     circle1Pos = circle1Pos_xz
        #     circle2Pos = circle2Pos_xz
        #     circle3Pos = circle3Pos_xz
        #     # Try camera 1
        # elif np.abs(dist1 - 3.2) <= thres:
        #     circle1Pos = circle1Pos_yz
        #     circle2Pos = circle2Pos_yz
        #     circle3Pos = circle3Pos_yz
        #     # If both camera fails
        #     # Get the closer of the two
        # else:
        #     if np.abs(dist1 - 3.2) <= np.abs(dist2 - 3.2):
        #         circle1Pos = circle1Pos_yz
        #         circle2Pos = circle2Pos_yz
        #         circle3Pos = circle3Pos_yz
        #     else:
        #         circle1Pos = circle1Pos_xz
        #         circle2Pos = circle2Pos_xz
        #         circle3Pos = circle3Pos_xz

        circle1Pos = circle1Pos_xz
        circle2Pos = circle2Pos_xz
        circle3Pos = circle3Pos_xz
        # Get the vector normalised by the length of the link 3
        norm_vector = (circle2Pos - circle1Pos) / 3.2

        # Get vector to zero_angle_red
        vector_control = norm_vector * 2.8

        # Compute the vector from current red to the blue
        vector_cur = circle3Pos - circle2Pos

        # Get orthogonal projection of where the red circle should be at 0 angle to the line
        c = np.dot(vector_cur, vector_control) / np.dot(vector_cur, vector_cur)
        orth = c * vector_cur
        orth_line = vector_control - orth

        # Calculate the angle
        ja4 = np.arctan2(np.sqrt(np.dot(orth_line, orth_line)), np.sqrt(np.dot(orth, orth)))

        # Need to adjust the angle to negative if it on the left side relative to the blue circle
        # First, define the orthogonal basis using vector_control as the new y-axis in the camera frame(i.e. z-axis(
        # and also the orthogonal vector to it.
        # Fix first coordinate as 1 and solve for 2nd
        vector_control_orth = np.array([1, vector_control[0] / vector_control[1]])

        # Convert the circle3Pos in terms of the new coordinate system
        circle3Pos_new_coord = np.dot(circle3Pos, vector_control_orth) / np.dot(vector_control_orth,
                                                                                vector_control_orth) * vector_control_orth + np.dot(
            circle3Pos, vector_control) / np.dot(vector_control, vector_control) * vector_control

        # # Check to see if it is on the left side relative to the new coordinate
        if circle3Pos_new_coord[0] < 0:
            ja4 = -ja4

        # Store last 10 ja1, ja3 and ja4 entries
        self.last_ten.append(np.array([ja1, ja3, ja4]))
        if len(self.last_ten) > 10:
            # Remove the oldest one
            self.last_ten.pop(0)

        # print(self.last_ten)
        return np.array([ja1, ja3, ja4])

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

        # Detecting the centre of the green circle

    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

        # Detecting the centre of the blue circle

    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        # print(M['m00'])
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

        # Detecting the centre of the yellow circle

    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

        # Calculate the conversion from pixel to meter

    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_green(image)
        circle2Pos = self.detect_yellow(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 4 / np.sqrt(dist)

# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


