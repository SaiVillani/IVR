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
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub2 = rospy.Publisher("joints_pos2", Float64MultiArray, queue_size=10)

        # # initialize a publisher to send joints' angular position to a topic called joints_pos
        # self.joints_pub2 = rospy.Publisher("joints_pos2", Float64MultiArray, queue_size=10)

        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback)

        # Flag to check if the circles are detected successfully
        self.red_flag = False
        self.blue_flag = False

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()


    # Recieve data from camera 1, process it, and publish
    def callback(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)

        # im1 = cv2.imshow('window1', self.cv_image1)

        # loading template for links as binary image
        # img1 = cv2.imread('./catkin_ws/src/ivr_assignment/src/link1.png', 1)
        # print(img1)
        # print(os.getcwd())
        self.link1 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link1.png', 1), (200, 200, 200), (255, 255, 255))
        self.link2 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link2.png', 1), (200, 200, 200), (255, 255, 255))
        self.link3 = cv2.inRange(cv2.imread('./catkin_ws/src/ivr_assignment/src/link3.png', 1), (200, 200, 200), (255, 255, 255))

        # publish robot joints angles
        self.joints = Float64MultiArray()

        # Get joint angles from camera 1
        joints_est1 = self.detect_joint_angles(self.cv_image1)

        # Get joint angles from camera 2
        joints_est2 = self.detect_joint_angles(self.cv_image2)

        # if self.red_flag == True:
        #     joint4_val = joints_est1[2]
        # else:
        #     joint4_val = joints_est2[2]

        # Blue not detected by camera 2
        if self.blue_flag:
            joints_est1[1] = -np.pi/2

        # Use camera 2 for estimating joint angles for 2 and 4
        # and camera 1 for estimating joint angles for 3

        # Manual adjustment if it exceeds pi/2 or -pi/2 in either directions
        self.joints.data = np.array([np.sign(joints_est2[1])*min(np.pi/2, abs(joints_est2[1])),
                                     np.sign(joints_est1[1])*min(np.pi/2, abs(joints_est1[1])),
                                     np.sign(joints_est2[2])*min(np.pi/2, abs(joints_est2[2]))])

        # Check why it is not publishing properly
        # Try Chamfer
        joints_est1_chamfer = self.detect_joint_angles_chamfer(self.cv_image1)
        joints_est2_chamfer = self.detect_joint_angles_chamfer(self.cv_image2)

        # if self.red_flag == True:
        #     joint4_val_chamfer = joints_est1_chamfer[2]
        # else:
        #     joint4_val_chamfer = joints_est2_chamfer[2]


        # publish robot joints angles
        self.joints_chamfer = Float64MultiArray()
        # self.joints_chamfer.data = np.array([joints_est2_chamfer[1], joints_est1_chamfer[1], joint4_val_chamfer])

        # Blue not detected by camera 2
        if self.blue_flag:
            joints_est1_chamfer[1] = -np.pi / 2

        # Manual adjustment if it exceeds pi/2 or -pi/2 in either directions
        self.joints_chamfer.data = np.array([np.sign(joints_est2_chamfer[1])*min(np.pi/2, abs(joints_est2_chamfer[1])),
                                            np.sign(joints_est1_chamfer[1])*min(np.pi/2, abs(joints_est1_chamfer[1])),
                                             np.sign(joints_est2_chamfer[2])*min(np.pi/2, abs(joints_est2_chamfer[2]))])

        # Publish the results
        try:
            # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            # self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.joints_pub.publish(self.joints)
        except CvBridgeError as e:
            print(e)
        try:
            self.joints_pub2.publish(self.joints_chamfer)
        except CvBridgeError as e:
            print(e)


    # # Recieve data from camera 1, process it, and publish
    # def callback1(self, data):
    #     # Recieve the image
    #     try:
    #         self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)
    #
    #     # Uncomment if you want to save the image
    #     # cv2.imwrite('image_copy.png', cv_image)
    #
    #     im1 = cv2.imshow('window1', self.cv_image1)
    #     cv2.waitKey(1)
    #
    #     # publish robot joints angles
    #     self.joints = Float64MultiArray()
    #     self.joints.data = self.detect_joint_angles(self.cv_image1)
    #
    #     # Publish the results
    #     try:
    #         self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    #         # self.robot_joint2_pub.publish(self.joint2)
    #         # self.robot_joint3_pub.publish(self.joint3)
    #         # self.robot_joint4_pub.publish(self.joint4)
    #         self.joints_pub.publish(self.joints)
    #     except CvBridgeError as e:
    #         print(e)
    #
    # def callback2(self, data):
    #     # Recieve the image
    #     try:
    #         self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)
    #     # Uncomment if you want to save the image
    #     # cv2.imwrite('image_copy.png', cv_image)
    #     im2 = cv2.imshow('window2', self.cv_image2)
    #     cv2.waitKey(1)
    #
    #     # publish robot joints angles
    #     self.joints2 = Float64MultiArray()
    #     self.joints2.data = self.detect_joint_angles(self.cv_image2)
    #
    #     # Publish the results
    #     try:
    #         # self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    #         # self.robot_joint2_pub.publish(self.joint2)
    #         # self.robot_joint3_pub.publish(self.joint3)
    #         # self.robot_joint4_pub.publish(self.joint4)
    #         self.joints_pub2.publish(self.joints2)
    #     except CvBridgeError as e:
    #         print(e)

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

    def detect_joint_angles(self, image):
        a = self.pixel2meter(image)
        # Obtain the centre of each coloured blob
        center = a * self.detect_green(image)
        circle1Pos = a * self.detect_yellow(image)

        # Solve using trigonometry
        ja1 = np.arctan2(center[0] - circle1Pos[0], center[1] - circle1Pos[1])

        try:
            circle2Pos = a * self.detect_blue(image)
            ja2 = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[1] - circle2Pos[1]) - ja1
            self.blue_flag = False
        except:
            # -90 degrees so it is completely hidden behind yellow
            ja2 = -np.pi/2
            circle2Pos = circle1Pos
            self.blue_flag = True

        try:
            circle3Pos = a * self.detect_red(image)
            ja3 = np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[1] - circle3Pos[1]) - ja2 - ja1
            self.red_flag = False
        except:
            ja3 = np.pi/2 - ja2 - ja1
            self.red_flag = True

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
    def detect_joint_angles_chamfer(self, image):
        # Obtain the center of each coloured blob
        center = self.detect_green(image)
        circle1Pos = self.detect_yellow(image)


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
                    ja2 = self.detect_l2(image, np.array([90, 180])) - ja1
                else:  # it is in right side
                    ja2 = self.detect_l2(image, np.array([0, 90])) - ja1
            else:  # it is in lower side
                if circle1Pos[0] - circle2Pos[0] >= 0:  # it is in left side
                    ja2 = self.detect_l2(image, np.array([180, 270])) - ja1
                else:  # it is in right side
                    ja2 = self.detect_l2(image, np.array([270, 360])) - ja1
        except:
            # -90 degrees so it is completely hidden behind yellow
            ja2 = -np.pi / 2
            circle2Pos = circle1Pos

        # link 3
        try:
            circle3Pos = self.detect_red(image)
            if circle2Pos[1] - circle3Pos[1] >= 0:  # it is in upper side
                if circle2Pos[0] - circle3Pos[0] >= 0:  # it is in left side
                    ja3 = self.detect_l3(image, np.array([90, 180])) - ja1 - ja2
                else:  # it is in right side
                    ja3 = self.detect_l3(image, np.array([0, 90])) - ja1 - ja2
            else:  # it is in lower side
                if circle2Pos[0] - circle3Pos[0] >= 0:  # it is in left side
                    ja3 = self.detect_l3(image, np.array([180, 270])) - ja1 - ja2
                else:  # it is in right side
                    ja3 = self.detect_l3(image, np.array([270, 360])) - ja1 - ja2
            self.red_flag = False
        except:
            ja3 = np.pi/2 - ja2 - ja1
            self.red_flag = True

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


