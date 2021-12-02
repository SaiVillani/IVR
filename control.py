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

    # initialize the bridge between openCV and ROS, 50Hz inverse signal
    self.bridge = CvBridge()
    rate = rospy.rate(50)

    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)

    # initialize a subscriber to receive camera1
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)

    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)

    # initialize a subscriber to receive camera2
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction", Float64MultiArray, queue_size=10)

    # initialize a publisher to send desired trajectory
    self.trajectory_pub = rospy.Publisher("trajectory", Float64MultiArray, queue_size=10)

    # initialize a publisher to send joints angles
    self.joint1_pub = rospy.Publisher("joint_angle_1", Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
    self.target_pos_pub = rospy.Publisher("target_pos", Float64MultiArray, queue_size=10)

    # initialize a publisher to send joints commands to arm
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

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

    self.b0 = 0
    self.b1 = 0
    self.b2 = 0





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


  # Calculate the forward kinematics
  def forward_kinematics(self, image1, image2):
      joints = self.detect_joint_angles(image1, image2)
      end_effector = np.array([2.8 * np.cos(joints[0]) * np.sin(joints[2]) + np.sin(joints[0]) * np.sin(
          joints[1]) * (2.8 * np.cos(joints[2]) + 3.2),
                               2.8 * np.sin(joints[0]) * np.sin(joints[2]) - np.cos(joints[0]) * np.sin(
                                   joints[1]) * (2.8 * np.cos(joints[2]) + 3.2),
                               np.cos(joints[1]) * (2.8 * np.cos(joints[2]) + 3.2) + 4.0])
      return end_effector

    #trajectory
  def trajectory(self):
      time = np.array([rospy.get_time()]) - self.t0
      self.time_x = float(3.0 * np.cos(time * np.pi /20))
      self.time_y = float(4.0 * np.sin(time * np.pi /14) + 0.5)
      self.time_z = float(1.0 * np.sin(time * np.pi / 18) + 4.5)
      return np.array([self.time_x, self.time_y, self.time_z])

  # Calculate the robot Jacobian
  def calculate_jacobian(self,image):
    joints = self.detect_joint_angles(image)
    jacobian = np.array([[-2.8 * np.sin(joints[0]) * np.sin(joints[2]) + np.cos(joints[0]) * np.sin(
        joints[1]) * (2.8 * np.cos(joints[2]) + 3.2),
                          np.sin(joints[0]) * np.cos(joints[1]) * (2.8 * np.cos(joints[2]) + 3.2),
                          2.8 * np.cos(joints[0]) * np.cos(joints[2]) + np.sin(joints[0]) * np.sin(
                              joints[1]) * (-2.8 * np.cos(joints[2]))],
                         [2.8 * np.cos(joints[0]) * np.sin(joints[2]) + np.sin(joints[0]) * np.sin(
                             joints[1]) * (2.8 * np.cos(joints[2]) + 3.2),
                          -np.cos(joints[0]) * np.cos(joints[1]) * (
                                  2.8 * np.cos(joints[2]) + 3.2),
                          2.8 * np.sin(joints[0]) * np.cos(joints[2]) - np.cos(joints[0]) * np.sin(
                              joints[1]) * (-2.8 * np.cos(joints[2]))],
                         [0, np.sin(joints[1]) * (2.8 * np.cos(joints[2]) + 3.2),
                          (-2.8 * np.cos(joints[2]))]])
    return jacobian

  # Estimate control inputs for open-loop control
  def control_open(self,image1, image2):
    # estimate time step
    time = rospy.get_time()
    time_drift = time - self.time_previous_step2
    self.time_previous_step2 = time
    start_pos = self.detect_joint_angles(image1, image2)

    jacobian_inverse = np.linalg.pinv(self.calculate_jacobian(image1, image2))  # calculating the psudeo inverse of Jacobian
    pos = np.array([self.ja1, self.ja2, self.ja3], dtype="float64")

    optimal_trajectory = self.trajectory()
    self.error = (optimal_trajectory - pos)/time_drift
    ja_trajectory = start_pos + (time_drift * np.dot(jacobian_inverse, self.error.transpose()))
    print(start_pos)
    print(jacobian_inverse)
    print(pos)
    print(optimal_trajectory)
    print(self.error)
    print(ja_trajectory)

    return ja_trajectory

  # Recieve data, process it, and publish
  def callback1(self,data):
      try:
          self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)
          # Publish the results
      try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      except CvBridgeError as e:
              print(e)
  def callback2(self,data):
      try:
          self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)

      a = self.detect_joint_angles(self.cv_image1, self.cv_image2)

      if ((self.b0 > 1 and a[0] < -1) or (self.b0 < -1 and a[0] > 1)):
          a[0] = -a[0]
      self.b0 = a[0]

      if ((self.b1 > 0.5 and a[1] < -0.5) or (self.b1 < -0.5 and a[1] > 0.5)):
             a[1] = -a[1]
      self.b1 = a[1]
      if ((self.b2 > 0.5 and a[2] < -0.5) or (self.b2 < -0.5 and a[2] > 0.5)):
          a[2] = -a[2]
      self.b2 = a[2]

      self.joint_angle_1 = Float64()
      self.joint_angle_1.data = a[0]
      self.joint_angle_3 = Float64()
      self.joint_angle_3.data = a[1]
      self.joint_angle_4 = Float64()
      self.joint_angle_4.data = a[2]

      # send control commands to joints
      q_d = self.control_open(self.cv_image1, self.cv_image2)
      self.joint1 = Float64()
      self.joint1.data = q_d[0]
      self.joint3 = Float64()
      self.joint3.data = q_d[1]
      self.joint4 = Float64()
      self.joint4.data = q_d[2]

      # compare the estimated position of robot end-effector calculated from images with forward kinematics
      x_e = self.forward_kinematics(self.cv_image1, self.cv_image2)
      x_e_image = np.array([self.red_x, self.red_y, self.red_z])
      self.end_effector = Float64MultiArray()
      self.end_effector.data = x_e_image

      # Publishing the desired trajectory on a topic named trajectory
      x_d = self.trajectory()  # getting the desired trajectory
      self.trajectory_desired = Float64MultiArray()
      self.trajectory_desired.data = x_d

      # Publish the results
      try:
          self.robot_joint1_pub.publish(self.joint1)
          self.robot_joint3_pub.publish(self.joint3)
          self.robot_joint4_pub.publish(self.joint4)
          self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
          self.joint1_pub.publish(self.joint_angle_1)
          self.joint3_pub.publish(self.joint_angle_3)
          self.joint4_pub.publish(self.joint_angle_4)
          self.end_effector_pub.publish(self.end_effector)
          self.trajectory_pub.publish(self.trajectory_desired)
      except CvBridgeError as e:
          print(e)

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
