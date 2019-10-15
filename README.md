# pyrealsense_location

This package arm_location provides a way to get 3D location of a pixel in a image by using Intel Realsense D435i camera. 

Using python driver of the camera, we can map depth points to the RGB image and get the pointe cloud which includes (x, y, z) coordinates for every pixel. 

In the arm_location/nodes, arm.csv is the file to record the location of a person's arms with the help of Vicon system, which is seen as the real location. In the meantime while the person moving her arms, the Realsense camera record the movements into a .bag file. With the help of the [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose), the arm location in the frames of the video can be obtained. /arm_location/node/test_trans.py is the method to map the pixels out and get the corresponding coordinates. 

The points can be visualized in the Rviz in ROS.


