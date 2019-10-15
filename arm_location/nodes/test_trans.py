#! /usr/bin/env python
import rospy
import sys
import tf
import cv2
import tf2_ros
import pandas as pd
import numpy as np
import geometry_msgs.msg
import pyrealsense2 as rs

from geometry_msgs.msg import TransformStamped


def camera_view():
    skeletons = np.load('/home/zyx/Downloads/skeletons.npy')
    # print(skeletons[0])
    # x,y pixel coordinates of point 4 and point 7
    x3 = (skeletons[:, 3, 0] * 640).astype(int)
    y3 = (skeletons[:, 3, 1] * 480).astype(int)
    x4 = (skeletons[:, 4, 0] * 640).astype(int)
    y4 = (skeletons[:, 4, 1] * 480).astype(int)
    x2 = (skeletons[:, 2, 0] * 640).astype(int)
    y2 = (skeletons[:, 2, 1] * 480).astype(int)

    c3 = list(zip(x3, y3))
    c4 = list(zip(x4, y4))
    c2 = list(zip(x2, y2))
    # print(c3)

	
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file("/home/zyx/Music/realsense.bag")
    profile = pipe.start(cfg)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # print('depth_scale:', depth_scale)
    a_to_e = []
    e_to_h = []
    w_to_a = []

    arm_points = []

    # calculate the initial point to the camera
    
    for i in range(0, 800):
        print('i:', i)
        frameset = pipe.wait_for_frames()

        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()
        #color_image = np.asanyarray(color_frame.get_data())
        #depth_image = np.asanyarray(depth_frame.get_data())

        # get point cloud and vertices in each pixel
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices())  # shape is (h*w,)
       
        vtx = vtx.view('<f4').reshape((480 * 640, 3))
        vtx = vtx.reshape(480, 640, 3)
        # print(np.shape(vtx))

        if i % 2 == 0:
            if c2[i/2][0] != -640 and c3[i/2][0] != -640 and c4[i/2][0] != -640:
                x2 = c2[i / 2][0]
                y2 = c2[i / 2][1]
                x3 = c3[i / 2][0]
                y3 = c3[i / 2][1]
                x4 = c4[i / 2][0]
                y4 = c4[i / 2][1]

                arm = vtx[x2, y2]

                elbow = vtx[x3, y3]
                hand = vtx[x4, y4]
                if abs(arm.any())==0:
                    continue
                if abs(elbow.any())==0:
                    continue
                if abs(hand.any())==0:
                    continue

                a_to_e.append(elbow - arm)
                e_to_h.append(hand - elbow)
                w_to_a.append(arm)

    print(len(w_to_a))
    print(len(e_to_h))
    pipe.stop()

    return w_to_a, a_to_e, e_to_h


def pub_trans(w_to_a, a_to_e, e_to_h):

    br = tf2_ros.TransformBroadcaster()
    # w_to_a, a_to_e, e_to_h = camera_view()
    print('starting publishing!')
    for i in range(len(a_to_e)):
        rospy.sleep(0.1)

        t1 = geometry_msgs.msg.TransformStamped()
        t1.header.frame_id = 'world'
        t1.child_frame_id = 'arm'
        w2a = tf.transformations.translation_matrix(w_to_a[i])
        Q1 = tf.transformations.quaternion_from_matrix(w2a)
        T1 = tf.transformations.translation_from_matrix(w2a)
        t1.transform.rotation.x = Q1[0]
        t1.transform.rotation.y = Q1[1]
        t1.transform.rotation.z = Q1[2]
        t1.transform.rotation.w = Q1[3]
        t1.transform.translation.x = round(T1[0], 2)
        t1.transform.translation.y = round(T1[1], 2)
        t1.transform.translation.z = round(T1[2], 2)
        br.sendTransform(t1)

        t2 = geometry_msgs.msg.TransformStamped()
        t2.header.frame_id = 'arm'
        t2.child_frame_id = 'elbow'
        a2e = tf.transformations.translation_matrix(a_to_e[i])
        Q2 = tf.transformations.quaternion_from_matrix(a2e)
        T2 = tf.transformations.translation_from_matrix(a2e)
        t2.transform.rotation.x = Q2[0]
        t2.transform.rotation.y = Q2[1]
        t2.transform.rotation.z = Q2[2]
        t2.transform.rotation.w = Q2[3]
        t2.transform.translation.x = round(T2[0], 2)
        t2.transform.translation.y = round(T2[1], 2)
        t2.transform.translation.z = round(T2[2], 2)
        br.sendTransform(t2)

        t3 = geometry_msgs.msg.TransformStamped()
        t3.header.frame_id = 'elbow'
        t3.child_frame_id = 'hand'
        e2h = tf.transformations.translation_matrix(e_to_h[i])
        Q3 = tf.transformations.quaternion_from_matrix(e2h)
        T3 = tf.transformations.translation_from_matrix(e2h)
        t3.transform.rotation.x = Q3[0]
        t3.transform.rotation.y = Q3[1]
        t3.transform.rotation.z = Q3[2]
        t3.transform.rotation.w = Q3[3]
        t3.transform.translation.x = round(T3[0], 2)
        t3.transform.translation.y = round(T3[1], 2)
        t3.transform.translation.z = round(T3[2], 2)
        br.sendTransform(t3)


if __name__ == '__main__':

    rospy.init_node('tr')

    rospy.sleep(0.1)
    # camera_view()
    w_to_a, a_to_e, e_to_h = camera_view()
    pub_trans(w_to_a, a_to_e, e_to_h)

    rospy.sleep(0.1)
