#! /usr/bin/env python3

import rospy
import cv2 
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

class dirve():

  pub = rospy.Publisher('/cmd_vel', String, queue_size=10)
  rospy.init_node('robot/camera/image_raw ', anonymous=True)
