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


class ImageConverter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # initialize a subscriber that receives joint angle estimates
    self.joint_angle_1 = rospy.Subscriber("joint_angle_1", Float64, self.callback)
    self.joint_angle_3 = rospy.Subscriber("joint_angle_3", Float64, self.callback)
    self.joint_angle_4 = rospy.Subscriber("joint_angle_4", Float64, self.callback)

    # initialize a publisher to send joints' controls forward kinematics
    self.robot_pub_ja1_fw = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_pub_ja3_fw = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_pub_ja4_fw = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # initialize a publisher to send joints' controls inverse kinematics
    self.robot_pub_ja1_inv = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_pub_ja3_inv = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_pub_ja4_inv = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # record the start time
    self.time_trajectory = rospy.get_time()

    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')

    # initialize error and derivative of error for trajectory tracking
    self.error = np.array([0.0, 0.0], dtype='float64')
    self.error_d = np.array([0.0, 0.0], dtype='float64')

    # Flag to check if the circles are detected successfully
    self.red_flag1 = False
    self.red_flag2 = False
    self.blue_flag1 = False
    self.blue_flag2 = False




  def detect_red(self,image):
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

  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 4 / np.sqrt(dist)

  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob
    center = a * self.detect_green(image)
    circle1Pos = a * self.detect_yellow(image)
    circle2Pos = a * self.detect_green(image)
    circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja1, ja2, ja3])

    # detect robot end-effector from the image

  def detect_end_effector(self,image):
    a = self.pixel2meter(image)
    endPos = a * (self.detect_yellow(image) - self.detect_red(image))
    return endPos


  # Calculate the forward kinematics
  def forward_kinematics(self,image):
    joints = self.detect_joint_angles(image)
    end_effector = np.array([3 * np.sin(joints[0]) + 3 * np.sin(joints[0]+joints[1]) + 3 *np.sin(joints.sum()),
                             3 * np.cos(joints[0]) + 3 * np.cos(joints[0]+joints[1]) + 3 * np.cos(joints.sum()),
                             3 * np.cos(joints[0]) + 3 * np.cos(joints[0]+joints[1]) + 3 * np.cos(joints.sum())])
    return end_effector

  # Calculate the robot Jacobian
  def calculate_jacobian(self,image):
    joints = self.detect_joint_angles(image)
    jacobian = np.array([[3 * np.cos(joints[0]) + 3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()),
                          3 * np.cos(joints[0]+joints[1]) + 3 *np.cos(joints.sum()),
                          3 *np.cos(joints.sum())], [-3 * np.sin(joints[0]) - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()),
                                                     - 3 * np.sin(joints[0]+joints[1]) - 3 * np.sin(joints.sum()),
                                                     - 3 * np.sin(joints.sum())]])
    return jacobian

  # Estimate control inputs for open-loop control
  def control_open(self,image):
    # estimate time step
    cur_time = rospy.get_time()
    dt = cur_time - self.time_previous_step2
    self.time_previous_step2 = cur_time
    q = self.detect_joint_angles(image) # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
    pos = self.detect_end_effector(image)
    # desired trajectory
    pos_d= self.trajectory()
    # estimate derivative of desired trajectory
    self.error = (pos_d - pos)/dt
    q_d = q + (dt * np.dot(J_inv, self.error.transpose()))  # desired joint angles to follow the trajectory
    return q_d

  # Recieve data, process it, and publish
  def callback(self,data):

    #Subscribe
    self.v1 = self.joint_angle_1
    self.v3 = self.joint_angle_3
    self.v4 = self.joint_angle_4
    print(self.v1)
    print(self.v3)
    print(self.v4)


# call the class
def main(args):
  ic = ImageConverter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
