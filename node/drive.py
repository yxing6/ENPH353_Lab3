#! /usr/bin/env python3

import rospy
import roslib
import sys
import cv2 as cv
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

    def image_callback(self, data):

        # try:
        #     # Convert the ROS Image message to OpenCV image
        #     cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        # except Exception as e:
        #     rospy.logerr(e)
        #     return

        # # Process the image (replace this with your image processing logic)
        # gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # kernel_size = 13
        # blur_gray = cv.GaussianBlur(gray,(kernel_size, kernel_size),5, 5)
        # ret, binary = cv.threshold(blur_gray, 70, 255, 0)
        # edges = cv.Canny(binary, 0, 200)
        # indices = [i for i, x in enumerate(edges[-1,:]) if x == 255]
        # linear_vel = 0.5  # m/s
        # angular_vel = 0.2  # rad/s

        # Create Twist message and publish to cmd_vel
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist_msg)

        # Optional: Display the processed image (for debugging purposes)
        # cv2.imshow('Processed Image', cv_image)
        # cv2.waitKey(1)

def main():
    try:
        image_subscriber = Drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
