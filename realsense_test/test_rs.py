#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import rospy
import cv_bridge
from sensor_msgs.msg import Image

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("/home/zyx/Downloads/object_detection.bag")

pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
rospy.init_node('publisher',anonymous=True)
profile = pipe.start(cfg)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
# print(depth_scale)
# Store next frameset for later processing:
while True:
    msg = cv_bridge.CvBridge()
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    color_msg = msg.cv2_to_imgmsg(color_image, encoding='bgr8')
    pc = rs.pointcloud()
    pc.map_to(color_frame)

    points = pc.calculate(depth_frame)


    vtx = np.asanyarray(points.get_vertices()) #shape is (h*w,)

    #tex = np.asanyarray(points.get_texture_coordinates())
    # blank_image = np.ones((921600,3), np.uint32)*255
    # vtx.convertTo(vtx,CV_8U, 255.0/1.0)
    #vtx = vtx.astype('uint32, uint32, uint32')
    vtx = vtx.view('<f4').reshape((921600,3))
    #print(np.bitwise_and(vtx, blank_image))
    #print(vtx[30000],vtx[600000])

    # for i in blank_image:
    #     vtx[i] = (i and vtx[i])
    print(cv2.resize(vtx.reshape((1280, 720,3)), (0,0), fx=0.5, fy=0.5) )
    #print(vtx[500000])

    #pub.publish(color_msg)
    # for i in range(0, points.size()):
    #     if vtx[i][2]:
    #         print(vtx[i][0])
    #         print(vtx[i][1])
pipe.stop()


