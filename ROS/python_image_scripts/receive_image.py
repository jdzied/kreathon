#! /usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node('ros_image_receiver')
rospy.loginfo('initialized node')

def callback(data):
    rospy.loginfo('Received image!')
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
    cv2.imshow('image', image)
    image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite('~/Bilder/received.jpg', image_bgr)
    return

def receive_image():
    rospy.Subscriber('/camera/image', Image, callback)

    rospy.loginfo('Waiting for image...')

    rospy.spin()

cv2.namedWindow('image')

while not rospy.is_shutdown():
    receive_image()

cv2.destroyAllWindows()
