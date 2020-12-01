#!/usr/bin/env python

import numpy as np
import rospy
import os
import cv2

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

rospy.init_node(rospy.get_param('/fakekinect/node_name'))

frame_id = rospy.get_param('/fakekinect/frame_id')

colorImagesPath = rospy.get_param('/fakekinect/color_images_path')
depthImagesPath = rospy.get_param('/fakekinect/depth_images_path')

image_pub = rospy.Publisher(rospy.get_param('/fakekinect/color_img_topic'), Image, queue_size=10)
depth_pub = rospy.Publisher(rospy.get_param('/fakekinect/depth_img_topic'), Image, queue_size=10)
info_pub = rospy.Publisher(rospy.get_param('/fakekinect/camera_info_topic'), CameraInfo, queue_size=10)

imageFileNames = os.listdir(colorImagesPath)
depthFileNames = os.listdir(depthImagesPath)
imageFileNames.sort()
depthFileNames.sort()

bridge = CvBridge()

assert(len(imageFileNames)==len(depthFileNames))

info_msg = CameraInfo()
info_msg.distortion_model = "plumb_bob"
info_msg.D = [0.05131238486536909, -0.0756855397919302, -0.0023319140234988587, 0.0001319931875445141, 0.023915458164988048]
info_msg.K = [1054.1971814582796, 0.0, 961.8791803107134, 0.0, 1053.0995786846784, 536.7196537517281, 0.0, 0.0, 1.0]
info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
info_msg.P = [1054.1971814582796, 0.0, 961.8791803107134, 0.0, 0.0, 1053.0995786846784, 536.7196537517281, 0.0, 0.0, 0.0, 1.0, 0.0]

for imageName,depthName in zip(imageFileNames,depthFileNames):
    image = cv2.imread(os.path.join(colorImagesPath, imageName))
    depth = cv2.imread(os.path.join(depthImagesPath, depthName),-1)

    info_msg.header.stamp = rospy.Time.now()
    info_msg.height, info_msg.width = image.shape[:2]

    image_pub.publish(bridge.cv2_to_imgmsg(image, encoding="bgr8"))
    depth_pub.publish(bridge.cv2_to_imgmsg(depth))
    info_pub.publish(info_msg)

    rospy.sleep(3)

