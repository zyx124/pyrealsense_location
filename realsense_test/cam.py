import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
# import rospy
import cv_bridge
import time


pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
cfg.enable_record_to_file('realsense.bag')


try:
    start = time.time()
    profile = pipe.start(cfg)
    while time.time() - start < 60:
        continue
    pipe.stop()
except:
    print('failed')
