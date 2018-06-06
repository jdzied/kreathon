#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

rospy.init_node('image_publisher_node')
rospy.logwarn('initialized node')

def publish_image_msg(img):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(img_rgb, encoding="rgb8")

    publisher = rospy.Publisher('/ipe/image', Image, queue_size=1)
    publisher.publish(msg)

    rospy.loginfo('Image sent')

def publish_compressed_image_msg(img):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

    publisher = rospy.Publisher('/camera/image/compressed', CompressedImage, queue_size=1)
    publisher.publish(msg)

    rospy.loginfo('Compressed image sent.')


def publish_image(filename):
    img = cv2.imread(filename)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    rospy.loginfo('image loaded')

    #publish_image_msg(img)
    publish_compressed_image_msg(img)

    return 0

file = "/home/ipeadm/Bilder/UsableThriftyMussaurus.jpg"

r = rospy.Rate(0.25) # 10hz
while not rospy.is_shutdown():
    publish_image(file)
    r.sleep()
