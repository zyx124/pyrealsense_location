import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import rospy
import cv_bridge


pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("/home/zyx/mobilebase/gazebo_sim/src/motion_follow/src/realsense.bag")
# cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
profile = pipe.start(cfg)

while True:

    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)

pipe.stop()