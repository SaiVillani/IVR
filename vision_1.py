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

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)

        # initialize publishers to send joints' angular position to topics
        self.joints_pub_ja2 = rospy.Publisher("joint_angle_2_v1", Float64, queue_size=10)
        self.joints_pub_ja3 = rospy.Publisher("joint_angle_3_v1", Float64, queue_size=10)
        self.joints_pub_ja4 = rospy.Publisher("joint_angle_4_v1", Float64, queue_size=10)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub2 = rospy.Publisher("joints_pos2", Float64MultiArray, queue_size=10)

        # initialize publishers to send joints' angular position to topics
        self.joints_pub2_ja2 = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        self.joints_pub2_ja3 = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joints_pub2_ja4 = rospy.Publisher("joint_angle_4", Float64, queue_size=10)

        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

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

        # publish robot joints angles
        self.joints = Float64MultiArray()

        # Get joint angles from camera 1
        # print('camera1')
        self.joints_est1 = self.detect_joint_angles1(self.cv_image1)

        # Chamfer
        self.joints_est1_chamfer = self.detect_joint_angles_chamfer1(self.cv_image1)

        # Blue not detected by camera 2
        # or Red not detected by camera 2
        # if (self.blue_flag2 == True) | (self.red_flag2 == True):
        #     self.joints_est1[1] = -np.pi/2
        #     self.joints_est1_chamfer[1] = -np.pi / 2

        # Case where camera 1 fails to detect blue ball
        if self.blue_flag1 == True:
            self.joints_est1[1] = 0
            self.joints_est1_chamfer[1] = 0

        # Publish the results
        try:
            self.joints_pub_ja3.publish(np.sign(self.joints_est1[1]) * min(np.pi / 2, abs(self.joints_est1[1])))
            self.joints_pub2_ja3.publish(np.sign(self.joints_est1_chamfer[1]) * min(np.pi / 2, abs(self.joints_est1_chamfer[1])))
        except CvBridgeError as e:
            print(e)

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

        # publish robot joints angles
        self.joints = Float64MultiArray()

        # Get joint angles from camera 2
        self.joints_est2 = self.detect_joint_angles2(self.cv_image2)

        # Chamfer
        self.joints_est2_chamfer = self.detect_joint_angles_chamfer2(self.cv_image2)
        #
        # print('Cam1', self.joints_est1_chamfer)
        # print('Cam2', self.joints_est2_chamfer)
        # print('Blue flag 1', self.blue_flag1)
        # print('Blue flag 2',self.blue_flag2)

        # Cases where camera 2 fails to detect red ball, switch camera
        if self.red_flag2 == True:
            self.joints_est2[2] = self.joints_est1[2]
            self.joints_est2_chamfer[2] = self.joints_est1_chamfer[2]

        # Publish the results
        try:
            self.joints_pub_ja2.publish(np.sign(self.joints_est2[1])*min(np.pi/2, abs(self.joints_est2[1])))
            self.joints_pub_ja4.publish(np.sign(self.joints_est2[2])*min(np.pi/2, abs(self.joints_est2[2])))
            self.joints_pub2_ja2.publish(np.sign(self.joints_est2_chamfer[1]) * min(np.pi / 2, abs(self.joints_est2_chamfer[1])))
            self.joints_pub2_ja4.publish(np.sign(self.joints_est2_chamfer[2]) * min(np.pi / 2, abs(self.joints_est2_chamfer[2])))
        except CvBridgeError as e:
            print(e)


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

        # Calculate the relevant joint angles from the image

    def detect_joint_angles1(self, image):
        a = self.pixel2meter(image)
        # Obtain the centre of each coloured blob
        center = a * self.detect_green(image)
        circle1Pos = a * self.detect_yellow(image)
        # print('This is center', center)
        # print('This is yellow',circle1Pos)

        # Solve using trigonometry
        ja1 = np.arctan2(center[0] - circle1Pos[0], center[1] - circle1Pos[1])

        try:
            circle2Pos = a * self.detect_blue(image)
            ja2 = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[1] - circle2Pos[1]) - ja1
            self.blue_flag1 = False
        except:
            # -90 degrees so it is completely hidden behind yellow
            ja2 = -np.pi/2
            circle2Pos = circle1Pos
            self.blue_flag1 = True

        try:
            circle3Pos = a * self.detect_red(image)
            ja3 = np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[1] - circle3Pos[1]) - ja2 - ja1
            self.red_flag1 = False
        except:
            ja3 = np.pi/2 - ja2 - ja1
            self.red_flag1 = True

        return np.array([ja1, ja2, ja3])

    def detect_joint_angles2(self, image):
        a = self.pixel2meter(image)
        # Obtain the centre of each coloured blob
        center = a * self.detect_green(image)
        circle1Pos = a * self.detect_yellow(image)
        # print('This is center', center)
        # print('This is yellow',circle1Pos)

        # Solve using trigonometry
        ja1 = np.arctan2(center[0] - circle1Pos[0], center[1] - circle1Pos[1])

        try:
            circle2Pos = a * self.detect_blue(image)
            ja2 = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[1] - circle2Pos[1]) - ja1
            self.blue_flag2 = False
        except:
            # -90 degrees for joint angle 3 so it is completely hidden behind yellow
            # For joint angle 2, use the current position of yellow ball and compare with
            # that when it is at 0 angle

            ja2 = -np.pi/2
            circle2Pos = circle1Pos
            self.blue_flag2 = True

        try:
            circle3Pos = a * self.detect_red(image)
            ja3 = np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[1] - circle3Pos[1]) - ja2 - ja1
            self.red_flag2 = False
        except:
            ja3 = np.pi/2 - ja2 - ja1
            self.red_flag2 = True

        return np.array([ja1, ja2, ja3])

    # In this method you can focus on detecting the rotation of link 1
    def detect_l1(self, image, quadrant):
        # find the center of the link
        circle1Pos = self.detect_green(image)
        circle2Pos = self.detect_yellow(image)
        center = (circle1Pos + circle2Pos) / 2

        # Isolate the region of interest in the thresholded image
        # (select an 160 by 160 window around the center of the link)
        mask = cv2.inRange(image, (0, 0, 0), (1, 1, 1))
        ROI = mask[int(center[1] - self.link1.shape[0] / 2): int(center[1] + self.link1.shape[0] / 2) + 1,
              int(center[0] - self.link1.shape[1] / 2): int(center[0] + self.link1.shape[1] / 2) + 1]
        ROI = ROI[0:self.link1.shape[0], 0:self.link1.shape[1]]  # making sure it has the same size as the template

        # Apply the distance transform
        dist = cv2.distanceTransform(cv2.bitwise_not(ROI), cv2.DIST_L2, 0)

        # rotate the template by small step sizes around the angles that was already estimated from lab 1 and compare
        # it with the cropped image of the link
        sumlist = np.array([])
        step = 1  # degree increment in the search
        rows, cols = self.link1.shape
        quadrant = quadrant - 90  # there is  90 degree difference between the robot frame and frame for rotating the
        # template
        angle_iteration = np.arange(quadrant[0], quadrant[1], step)
        for i in angle_iteration:
            # Rotate the template to the desired rotation configuration
            M = cv2.getRotationMatrix2D((cols / 2, rows / 2), i, 1)
            # Apply rotation to the template
            rotatedTemplate = cv2.warpAffine(self.link1, M, (cols, rows))
            # Combine the template and region of interest together to obtain only the values that are inside the template
            img = dist * rotatedTemplate
            # Sum the distances and append to the list
            sumlist = np.append(sumlist, np.sum(img))

        # Once all configurations have been searched then select the one with the smallest distance and convert
        # to radians.
        return (angle_iteration[np.argmin(sumlist)] * np.pi) / 180.0

    # In this method you can focus on detecting the rotation of link 2
    def detect_l2(self, image, quadrant):
        # find the center of the link
        circle1Pos = self.detect_yellow(image)
        circle2Pos = self.detect_blue(image)
        center = (circle1Pos + circle2Pos) / 2
        # center.astype(int)

        # Isolate the region of interest in the thresholded image
        # (select an 160 by 160 window around the center of the link)
        mask = cv2.inRange(image, (0, 0, 0), (1, 1, 1))
        ROI = mask[int(center[1] - self.link2.shape[0] / 2): int(center[1] + self.link2.shape[0] / 2) + 1,
              int(center[0] - self.link2.shape[1] / 2): int(center[0] + self.link2.shape[1] / 2) + 1]
        ROI = ROI[0:self.link2.shape[0], 0:self.link2.shape[1]]  # making sure it has the same size as the template

        # Apply the distance transform
        dist = cv2.distanceTransform(cv2.bitwise_not(ROI), cv2.DIST_L2, 0)

        # rotate the template by small step sizes around the angles that was already estimated from lab 1 and compare
        # it with the cropped image of the link
        sumlist = np.array([])
        step = 1  # degree increment in the search
        rows, cols = self.link2.shape
        quadrant = quadrant - 90  # there is  90 degree difference between the robot frame and frame for rotating the
        # template
        angle_iteration = np.arange(quadrant[0], quadrant[1], step)
        for i in angle_iteration:
            # Rotate the template to the desired rotation configuration
            M = cv2.getRotationMatrix2D((cols / 2, rows / 2), i, 1)
            # Apply rotation to the template
            rotatedTemplate = cv2.warpAffine(self.link1, M, (cols, rows))
            # Combine the template and region of interest together to obtain only the values that are inside the template
            img = dist * rotatedTemplate
            # Sum the distances and append to the list
            sumlist = np.append(sumlist, np.sum(img))

        # Once all configurations have been searched then select the one with the smallest distance and convert
        # to radians.
        return (angle_iteration[np.argmin(sumlist)] * np.pi) / 180.0

    # In this method you can focus on detecting the rotation of link 3
    def detect_l3(self, image, quadrant):
        # find the center of the link
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_red(image)
        center = (circle1Pos + circle2Pos) / 2

        # Isolate the region of interest in the thresholded image
        # (select an 160 by 160 window around the center of the link)
        mask = cv2.inRange(image, (0, 0, 0), (1, 1, 1))
        ROI = mask[int(center[1] - self.link3.shape[0] / 2): int(center[1] + self.link3.shape[0] / 2) + 1,
              int(center[0] - self.link3.shape[1] / 2): int(center[0] + self.link3.shape[1] / 2) + 1]
        ROI = ROI[0:self.link3.shape[0], 0:self.link3.shape[1]]  # making sure it has the same size as the template

        # Apply the distance transform
        dist = cv2.distanceTransform(cv2.bitwise_not(ROI), cv2.DIST_L2, 0)

        # rotate the template by small step sizes around the angles that was already estimated from lab 1 and compare
        # it with the cropped image of the link
        sumlist = np.array([])
        step = 1  # degree increment in the search
        rows, cols = self.link3.shape
        quadrant = quadrant - 90  # there is  90 degree difference between the robot frame and frame for rotating the
        # template
        angle_iteration = np.arange(quadrant[0], quadrant[1], step)
        for i in angle_iteration:
            # Rotate the template to the desired rotation configuration
            M = cv2.getRotationMatrix2D((cols / 2, rows / 2), i, 1)
            # Apply rotation to the template
            rotatedTemplate = cv2.warpAffine(self.link1, M, (cols, rows))
            # Combine the template and region of interest together to obtain only the values that are inside the template
            img = dist * rotatedTemplate
            # Sum the distances and append to the list
            sumlist = np.append(sumlist, np.sum(img))

        # Once all configurations have been searched then select the one with the smallest distance and convert
        # to radians.
        return (angle_iteration[np.argmin(sumlist)] * np.pi) / 180.0

    # Calculate the relevant joint angles from the image
    def detect_joint_angles_chamfer1(self, image):
        # Obtain the center of each coloured blob
        center = self.detect_green(image)
        circle1Pos = self.detect_yellow(image)


        # Determine which quadrant each link is pointing in and detect the angle
        # link 1
        if center[0] - circle1Pos[0] >= 0:
            ja1 = -self.detect_l1(image, np.array([90, 180]))  # it is in left side
        else:
            ja1 = self.detect_l1(image, np.array([0, 90]))  # it is in right side

        # link 2
        try:
            circle2Pos = self.detect_blue(image)
            if circle1Pos[1] - circle2Pos[1] >= 0:  # it is in upper side
                if circle1Pos[0] - circle2Pos[0] >= 0:  # it is in left side
                    ja2 = self.detect_l2(image, np.array([90, 180]))
                    # print(ja2)
                    # print('In left side')
                    # print(self.detect_l2(image, np.array([90, 180])))
                else:  # it is in right side
                    ja2 = self.detect_l2(image, np.array([0, 90])) #- ja1
                    # print(ja2)
                    # print('In right side')
            else:  # it is in lower side
                if circle1Pos[0] - circle2Pos[0] >= 0:  # it is in left side
                    ja2 = -(self.detect_l2(image, np.array([180, 270])) - np.pi) # self.detect_l2(image, np.array([180, 270])) #- ja1
                    # print(ja2)
                else:  # it is in right side
                    ja2 = -(np.pi/2 - (np.pi*3/2 - self.detect_l2(image, np.array([270, 360]))))#-self.detect_l2(image, np.array([270, 360])) #- ja1
                    # print(ja2)
            self.blue_flag1 = False
        except:
            # Note that this joint angle is for joint angle 3
            # Only a fleeting second that this happens
            # Heuristically, we know that happens for joint angle 3 around 0
            # Will use minus joint angle 1 as estimation (since the rotation angle is inverse)
            # right side is equivalent to negative angle of rotation
            ja2 = -ja1
            circle2Pos = circle1Pos
            self.blue_flag1 = True
            # print('joint 2 failed')

        # link 3
        try:
            # Try to formulate a reference frame
            # Idea: Get linear "extrapolation" using yellow circle and the blue extending to
            # where the red circle should be if it were 0 angle.
            circle3Pos = self.detect_red(image)

            # Get the vector normalised by the length of the link 3
            norm_vector = (circle2Pos - circle1Pos)/3.2

            # Get vector to zero_angle_red
            vector_control = norm_vector * 2.8

            # Compute the vector from current red to the blue
            vector_cur = circle3Pos - circle2Pos

            # Get orthogonal projection of where the red circle should be at 0 angle to the line
            c = np.dot(vector_cur, vector_control)/np.dot(vector_cur, vector_cur)
            orth = c * vector_cur
            orth_line = vector_control - orth

            # Calculate the angle
            ja3 = np.arctan2(np.sqrt(np.dot(orth_line, orth_line)), np.sqrt(np.dot(orth, orth)))

            # Need to adjust the angle to negative if it on the left side relative to the blue circle
            # First, define the orthogonal basis using vector_control as the new y-axis in the camera frame(i.e. z-axis(
            # and also the orthogonal vector to it.
            # Fix first coordinate as 1 and solve for 2nd
            vector_control_orth = np.array([1, vector_control[0]/vector_control[1]])

            # Convert the circle3Pos in terms of the new coordinate system
            circle3Pos_new_coord = np.dot(circle3Pos, vector_control_orth)/np.dot(vector_control_orth, vector_control_orth) * vector_control_orth + np.dot(circle3Pos,vector_control)/np.dot(vector_control,vector_control) * vector_control

            # # Check to see if it is on the right side relative to the new coordinate
            if circle3Pos_new_coord[0] > 0:
                ja3 = -ja3

            if np.abs(ja3) > 3*np.pi/2:
                ja3 = np.sign(ja3) * (ja3 - 3*np.pi/2)
            elif np.abs(ja3) > np.pi:
                ja3 = np.sign(ja3) * (3*np.pi/2 -ja3)
            elif np.abs(ja3) > np.pi/2:
                ja3 = np.sign(ja3) * (np.pi -ja3)
            else:
                ja3 = ja3

            # print(ja3)
            # print(ja3)
            # if circle2Pos[1] - circle3Pos[1] >= 0:  # it is in upper side
            #     if circle2Pos[0] - circle3Pos[0] >= 0:  # it is in left side
            #         ja3 = self.detect_l3(image, np.array([90, 180])) - ja2 #- ja1
            #         # print('joint 3 upper left side')
            #     else:  # it is in right side
            #         ja3 = self.detect_l3(image, np.array([0, 90])) - ja2 #- ja1
            #         # print('joint 3 upper right side')
            # else:  # it is in lower side
            #     if circle2Pos[0] - circle3Pos[0] >= 0:  # it is in left side
            #         ja3 = -(self.detect_l2(image, np.array([180, 270])) - np.pi) - ja2 #- ja1
            #         # print('joint 3 lower left side')
            #     else:  # it is in right side
            #         ja3 = -(np.pi*3/2 - self.detect_l2(image, np.array([270, 360])))- ja2 #- ja1
            #         # print('joint 3 lower right side')
            self.red_flag1 = False
        except:
            ja3 = np.pi/2 - ja2 - ja1
            self.red_flag1 = True

        # print(ja3)
        return np.array([ja1, ja2, ja3])

    # Calculate the relevant joint angles from the image
    def detect_joint_angles_chamfer2(self, image):
        # Obtain the center of each coloured blob
        center = self.detect_green(image)
        circle1Pos = self.detect_yellow(image)
        # print(center)
        # print(circle1Pos)

        # Determine which quadrant each link is pointing in and detect the angle
        # link 1
        if center[0] - circle1Pos[0] >= 0:
            ja1 = self.detect_l1(image, np.array([90, 180]))  # it is in left side
        else:
            ja1 = self.detect_l1(image, np.array([0, 90]))  # it is in right side

        # link 2
        try:
            circle2Pos = self.detect_blue(image)
            if circle1Pos[1] - circle2Pos[1] >= 0:  # it is in upper side
                if circle1Pos[0] - circle2Pos[0] >= 0:  # it is in left side
                    ja2 = -self.detect_l2(image, np.array([90, 180]))
                    # print(ja2)
                    # print('In left side')
                    # print(self.detect_l2(image, np.array([90, 180])))
                else:  # it is in right side
                    ja2 = -self.detect_l2(image, np.array([0, 90])) #- ja1
                    # print(ja2)
                    # print('In right side')
            else:  # it is in lower side
                if circle1Pos[0] - circle2Pos[0] >= 0:  # it is in left side
                    ja2 = (self.detect_l2(image, np.array([180, 270])) - np.pi)#- ja1
                    # print(ja2)
                else:  # it is in right side
                    ja2 = np.pi/2 - (np.pi*3/2 - self.detect_l2(image, np.array([270, 360]))) #- ja1
                    # print(ja2)

            # print(ja2)
            self.blue_flag2 = False
        except:
            # Note that this joint angle might be joint angle
            # For camera angle 2
            circle1_originalPos = np.array([320, 431])
            circle2Pos = circle1Pos
            ja2 = np.arctan2(circle1_originalPos[0] - circle1Pos[0], circle1_originalPos[1] - circle1Pos[1])

            # print(circle2Pos)
            # print(circle1_originalPos)
            if circle1_originalPos[1] - circle2Pos[1] >= 0:  # it is in upper side
                if circle1_originalPos[0] - circle2Pos[0] >= 0:  # it is in left side
                    ja2 = ja2
                    # print('upper left')
                else:  # it is in right side
                    ja2 = ja2
                    # print('upper right')
                    # print(ja2)
            else:  # it is in lower side
                if circle1_originalPos[0] - circle2Pos[0] >= 0:  # it is in left side
                    ja2 = (ja2 - np.pi)  # - ja1
                    # print('lower left')
                else:  # it is in right side
                    ja2 = -(np.pi/2 + ja2)  # - ja1
                    # print('lower right')
            # print(ja2)
            self.blue_flag2 = True
            # print('joint 2 failed')

        # link 3
        try:
            # Try to formulate a reference frame
            # Idea: Get linear "extrapolation" using yellow circle and the blue extending to
            # where the red circle should be if it were 0 angle.
            circle3Pos = self.detect_red(image)

            # Get the vector normalised by the length of the link 3
            norm_vector = (circle2Pos - circle1Pos)/3.2

            # Get vector to zero_angle_red
            vector_control = norm_vector * 2.8

            # Compute the vector from current red to the blue
            vector_cur = circle3Pos - circle2Pos

            # Get orthogonal projection of where the red circle should be at 0 angle to the line
            c = np.dot(vector_cur, vector_control)/np.dot(vector_cur, vector_cur)
            orth = c * vector_cur
            orth_line = vector_control - orth

            # Calculate the angle
            ja3 = np.arctan2(np.sqrt(np.dot(orth_line, orth_line)), np.sqrt(np.dot(orth, orth)))

            # Need to adjust the angle to negative if it on the left side relative to the blue circle
            # First, define the orthogonal basis using vector_control as the new y-axis in the camera frame(i.e. z-axis(
            # and also the orthogonal vector to it.
            # Fix first coordinate as 1 and solve for 2nd
            vector_control_orth = np.array([1, vector_control[0]/vector_control[1]])

            # Convert the circle3Pos in terms of the new coordinate system
            circle3Pos_new_coord = np.dot(circle3Pos, vector_control_orth)/np.dot(vector_control_orth, vector_control_orth) * vector_control_orth + np.dot(circle3Pos,vector_control)/np.dot(vector_control,vector_control) * vector_control

            # # Check to see if it is on the left side relative to the new coordinate
            if circle3Pos_new_coord[0] < 0:
                ja3 = -ja3

            if np.isnan(circle3Pos_new_coord[0]):
                self.red_flag2 = True
            else:
                self.red_flag2 = False
            # print(np.sign(circle3Pos_new_coord[0]))
            # print(ja3)
            # if np.isnan(circle3Pos_new_coord[0]):
            #     print('Backup triggered')
            #     if circle2Pos[1] - circle3Pos[1] >= 0:  # it is in upper side
            #         if circle2Pos[0] - circle3Pos[0] >= 0:  # it is in left side
            #             ja3 = -self.detect_l3(image, np.array([90, 180])) - ja2
            #             # print(ja3)
            #             # print('joint 3 upper left side')
            #         else:  # it is in right side
            #             ja3 = -self.detect_l3(image, np.array([0, 90])) - ja2
            #             # print(ja3)
            #             # print('joint 3 upper right side')
            #     else:  # it is in lower side
            #         if circle2Pos[0] - circle3Pos[0] >= 0:  # it is in left side
            #             ja3 = (self.detect_l3(image, np.array([180, 270])) - np.pi) - ja2
            #             # print(ja3)
            #             # print('joint 3 lower left side')
            #         else:  # it is in right side
            #             ja3 = np.pi/2 - (np.pi*3/2 - self.detect_l3(image, np.array([270, 360]))) - ja2
            #             # print(ja3)
            #             # print('joint 3 lower right side')

        except:
            # Temp fix
            ja3 = np.pi/2 - ja2 - ja1
            # print(ja3)
            self.red_flag2 = True

        return np.array([ja1, ja2, ja3])
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


