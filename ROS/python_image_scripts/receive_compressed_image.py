#! /usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

rospy.init_node('ros_image_receiver')
rospy.loginfo('initialized node')

def callback(data):
    rospy.loginfo('Received image!')
    bridge = CvBridge()
    np_arr = np.fromstring(data.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    rospy.loginfo('Image shape: ' + str(image.shape))
    image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    image_path = '/home/ipeadm/Bilder/received.jpg'
    cv2.imwrite(image_path, image_bgr)
    rospy.loginfo('Wrote image to ' + image_path)
    cv2.imshow('image', image)
    return

def receive_image():
    rospy.Subscriber('/camera/image/compressed', CompressedImage, callback)

    rospy.loginfo('Waiting for image...')

    rospy.spin()

cv2.namedWindow('image')

while not rospy.is_shutdown():
    receive_image()

cv2.destroyAllWindows()

