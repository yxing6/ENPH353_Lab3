#! /usr/bin/env python3

import rospy
import roslib
import sys
import cv2 as cv 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Drive:

    def __init__(self):

        rospy.init_node('image_subscriber_node', anonymous=True)

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.image_callback)

        # Create a publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # control parameters
        self.Kp = 0.02 # Proportional gain
        self.linear_val_max = 0.6
        self.linear_val_min = 0.2
        self.error_threshold = 20
        self.mid_x = 0

    def image_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            rospy.logerr(e)
            return

        angular_error = self.detect_line(cv_image)

        print(angular_error)
        if abs(angular_error) <= self.error_threshold:
            linear_vel = self.linear_val_max
        else:
            linear_vel = self.linear_val_min
        # linear_vel = 0.2
        angular_vel = self.Kp * angular_error

        # Create Twist message and publish to cmd_vel
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist_msg)


    def detect_line(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        kernel_size = 13
        blur_gray = cv.GaussianBlur(gray,(kernel_size, kernel_size),5, 5)
        ret, binary = cv.threshold(blur_gray, 70, 255, 0)
        edges = cv.Canny(binary, 0, 200)
        cv.imshow("name", binary)
        cv.waitKey(3)
        last_row = binary[-1,:]
        # print(last_row)
        # print(binary.type())

        # target_row = edges[-30,:]
        print(last_row)

        
        new_mid_x = self.find_mid_binary(last_row, self.mid_x)
        self.mid_x = new_mid_x

        mid_frame = len(last_row) / 2
        error = mid_frame - new_mid_x

        return error

    
    def find_mid_binary(self, bottom_row, previous_mid):
        
        if np.any(bottom_row == 0):
            bottom_list = bottom_row.tolist()
            print(bottom_list)
            first_index = bottom_list.index(0)
            last_index = len(bottom_list) - 1 - bottom_list[::-1].index(0)
            new_mid = (first_index + last_index)/2
            print(first_index, new_mid, last_index)
        else:
            new_mid = previous_mid

        return new_mid


def main():
    try:
        image_subscriber = Drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
