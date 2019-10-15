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

# read csv and calculate the center of elbows and hands

def calculate():
    df = pd.read_csv('/home/zyx/catkin_ws/src/arm_location/nodes/arm.csv', skiprows=[0,1,2,4])
    i=2
    left_elbow_x = (df.iloc[:, i] + df.iloc[:, i+3] + df.iloc[:, i+6]) / 3
    left_elbow_y = (df.iloc[:, i+1] + df.iloc[:, i+4] + df.iloc[:, i+7]) / 3
    left_elbow_z = (df.iloc[:, i+2] + df.iloc[:, i+5] + df.iloc[:, i+8]) / 3
    left_hand_x = (df.iloc[:, i+9] + df.iloc[:, i+12] + df.iloc[:, i+15]) / 3
    left_hand_y = (df.iloc[:, i+10] + df.iloc[:, i+13] + df.iloc[:, i+16]) / 3
    left_hand_z = (df.iloc[:, i+11] + df.iloc[:, i+14] + df.iloc[:, i+17]) / 3

    j=20
    right_elbow_x = (df.iloc[:, j] + df.iloc[:, j+3] + df.iloc[:, j+6]) / 3
    right_elbow_y = (df.iloc[:, j+1] + df.iloc[:, j+4] + df.iloc[:, j+7]) / 3
    right_elbow_z = (df.iloc[:, j+2] + df.iloc[:, j+5] + df.iloc[:, j+8]) / 3
    right_hand_x = (df.iloc[:, j + 9] + df.iloc[:, j + 12] + df.iloc[:, j + 15]) / 3
    right_hand_y = (df.iloc[:, j + 10] + df.iloc[:, j + 13] + df.iloc[:, j + 16]) / 3
    right_hand_z = (df.iloc[:, j + 11] + df.iloc[:, j + 14] + df.iloc[:, j + 17]) / 3
    return left_elbow_x/1000, left_elbow_y/1000, left_elbow_z/1000, left_hand_x/1000, left_hand_y/1000, left_hand_z/1000, right_elbow_x/1000, right_elbow_y/1000, right_elbow_z/1000, right_hand_x/1000, right_hand_y/1000, right_hand_z/1000


def camera_view():
    skeletons = np.load('/home/zyx/Downloads/skeletons.npy')
    # print(skeletons[0])
    # x,y pixel coordinates of point 4 and point 7
    x4 = (skeletons[:, 4, 0] * 640).astype(int)
    y4 = (skeletons[:, 4, 1] * 480).astype(int)
    x7 = (skeletons[:, 7, 0] * 640).astype(int)
    y7 = (skeletons[:, 7, 1] * 480).astype(int)
    c4 = list(zip(x4, y4))
    c7 = list(zip(x7, y7))

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file("/home/zyx/Music/realsense.bag")
    profile = pipe.start(cfg)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # print('depth_scale:', depth_scale)
    p_to_c = 0
    j = 0

    # calculate the initial point to the camera

    for i in range(10):
        frameset = pipe.wait_for_frames()

        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # get point cloud and vertices in each pixel
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices())  # shape is (h*w,)
        # print(np.shape(vtx))
        vtx = vtx.view('<f4').reshape((480 * 640, 3))
        # print(np.shape(vtx))
        vtx = vtx.reshape(480, 640, 3)
        if c4[i][0] != -640 and i <= 10:
            j = j + 1
            p_to_c = p_to_c + vtx[c4[i][0], c4[i][1]]

    pipe.stop()
    print(j)
    return p_to_c / j

def pub_transforms():
    p_to_c = camera_view()
    lex, ley, lez, lhx, lhy, lhz, rex, rey, rez, rhx, rhy, rhz = calculate()
    # print(lex-rex)
    br = tf2_ros.TransformBroadcaster()
    print('starting publishing')
    for i in range(len(lex)):
        rospy.sleep(0.01)

        t1 = geometry_msgs.msg.TransformStamped()
        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = 'world'
        t1.child_frame_id = 'left_elbow'
        w2le = tf.transformations.translation_matrix([lex[i], ley[i], lez[i]])
        Q1 = tf.transformations.quaternion_from_matrix(w2le)
        T1 = tf.transformations.translation_from_matrix(w2le)
        t1.transform.rotation.x = Q1[0]
        t1.transform.rotation.y = Q1[1]
        t1.transform.rotation.z = Q1[2]
        t1.transform.rotation.w = Q1[3]
        t1.transform.translation.x = T1[0]
        t1.transform.translation.y = T1[1]
        t1.transform.translation.z = T1[2]
        br.sendTransform(t1)

        # t2 = geometry_msgs.msg.TransformStamped()
        # t2.header.stamp = rospy.Time.now()
        # t2.header.frame_id = 'world'
        # t2.child_frame_id = 'right_elbow'
        # w2re = tf.transformations.translation_matrix([rex[i], rey[i], rez[i]])
        # Q2 = tf.transformations.quaternion_from_matrix(w2re)
        # T2 = tf.transformations.translation_from_matrix(w2re)
        # t2.transform.rotation.x = Q2[0]
        # t2.transform.rotation.y = Q2[1]
        # t2.transform.rotation.z = Q2[2]
        # t2.transform.rotation.w = Q2[3]
        # t2.transform.translation.x = T2[0]
        # t2.transform.translation.y = T2[1]
        # t2.transform.translation.z = T2[2]
        # br.sendTransform(t2)
        #
        t3 = geometry_msgs.msg.TransformStamped()
        t3.header.stamp = rospy.Time.now()
        t3.header.frame_id = 'left_elbow'
        t3.child_frame_id = 'left_hand'
        lh2le = tf.transformations.translation_matrix([lhx[i]-lex[i], lhy[i]-ley[i], lhz[i]-lez[i]])
        Q3 = tf.transformations.quaternion_from_matrix(lh2le)
        T3 = tf.transformations.translation_from_matrix(lh2le)
        t3.transform.rotation.x = Q3[0]
        t3.transform.rotation.y = Q3[1]
        t3.transform.rotation.z = Q3[2]
        t3.transform.rotation.w = Q3[3]
        t3.transform.translation.x = T3[0]
        t3.transform.translation.y = T3[1]
        t3.transform.translation.z = T3[2]
        br.sendTransform(t3)
        #
        # t4 = geometry_msgs.msg.TransformStamped()
        # t4.header.frame_id = 'right_elbow'
        # t4.child_frame_id = 'right_hand'
        # re2rh = tf.transformations.translation_matrix([rhx[i]-rex[i], rhy[i]-rey[i], rhz[i]-rez[i]])
        # Q4 = tf.transformations.quaternion_from_matrix(re2rh)
        # T4 = tf.transformations.translation_from_matrix(re2rh)
        # t4.transform.rotation.x = Q4[0]
        # t4.transform.rotation.y = Q4[1]
        # t4.transform.rotation.z = Q4[2]
        # t4.transform.rotation.w = Q4[3]
        # t4.transform.translation.x = T4[0]
        # t4.transform.translation.y = T4[1]
        # t4.transform.translation.z = T4[2]
        # br.sendTransform(t4)

        # t5 = geometry_msgs.msg.TransformStamped()
        # t5.header.frame_id = "world"
        # t5.child_frame_id = 'camera'
        # c2rh0 = tf.transformations.translation_matrix([p_to_c[0], p_to_c[1], p_to_c[2]])
        # # print('c2rh:', c2rh0)
        #
        # w2rh0 = tf.transformations.translation_matrix([(rhx[0]+rhx[1]+rhx[2])/3, (rhy[0]+rhy[1]+rhy[2])/3, (rhz[0]+rhz[1]+rhz[2])/3])
        # rh02c = tf.transformations.inverse_matrix(c2rh0)
        # w2c = tf.transformations.concatenate_matrices(w2rh0, rh02c)
        # # print('w2rh:',w2rh0)
        # # print('rh2c:', rh02c)
        # # print(w2c)
        # Q5 = tf.transformations.quaternion_from_matrix(w2c)
        # T5 = tf.transformations.translation_from_matrix(w2c)
        # t5.transform.rotation.x = Q5[0]
        # t5.transform.rotation.y = Q5[1]
        # t5.transform.rotation.z = Q5[2]
        # t5.transform.rotation.w = Q5[3]
        # t5.transform.translation.x = T5[0]
        # t5.transform.translation.y = T5[1]
        # t5.transform.translation.z = T5[2]
        #
        # br.sendTransform(t5)



if __name__ == '__main__':

    rospy.init_node('tr')

    rospy.sleep(0.1)

    pub_transforms()
    print('finished')
    rospy.spin()
