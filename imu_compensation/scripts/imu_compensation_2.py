#!/usr/bin/env python3

from dataclasses import fields
import rospy 
from sensor_msgs.msg import Imu, PointCloud2
import transforms3d as tf
import numpy as np
import message_filters as mf


def imu_callback(msg, lidar_msg):
  
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w

    orient = tf.quaternions.quat2mat([w,x,y,z])
    R = np.array([[1,0,0], [0, -1, 0], [0, 0, -1]])
    transform_rot_mat = np.matmul(orient, R)
    transform_quat = tf.quaternions.mat2quat(transform_rot_mat)

    # w_tf = transform_quat[0]
    # x_tf = transform_quat[1]
    # y_tf = transform_quat[2]
    # z_tf = transform_quat[3]

    # ang_x = msg.angular_velocity.x
    
    # ang_y = msg.angular_velocity.y
    # ang_z = msg.angular_velocity.z

    # acc_x = msg.linear_acceleration.x
    # acc_y = msg.linear_acceleration.y
    # acc_z = msg.linear_acceleration.z

    pub2 = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=200)
    pub2.publish(lidar_msg)
    pub= rospy.Publisher('/imu/imu_compn', Imu, queue_size=200)
  

    imu_msg = Imu()
    
    imu_msg.header.seq = msg.header.seq
    imu_msg.header.stamp = msg.header.stamp
    imu_msg.header.frame_id = 'Imu_compensated'

    imu_msg.orientation.x = transform_quat[0]
    imu_msg.orientation.y = transform_quat[1]
    imu_msg.orientation.z = transform_quat[2]
    imu_msg.orientation.w = transform_quat[3]

    imu_msg.angular_velocity.x = msg.angular_velocity.x 
    imu_msg.angular_velocity.y =  -msg.angular_velocity.y 
    imu_msg.angular_velocity.z =  -msg.angular_velocity.z 


    imu_msg.linear_acceleration.x = msg.linear_acceleration.x 
    imu_msg.linear_acceleration.y = -msg.linear_acceleration.y  
    imu_msg.linear_acceleration.z = -msg.linear_acceleration.z 

    pub.publish(imu_msg)
    
    


if __name__ == '__main__':
    rospy.init_node("imu_compensator")

    imu_sub = mf.Subscriber("/imu/imu", Imu)
    lidar_sub = mf.Subscriber("/ns1/velodyne_points", PointCloud2)
    ts = mf.ApproximateTimeSynchronizer([imu_sub, lidar_sub], 10, 1, allow_headerless=False)
    ts.registerCallback(imu_callback)
    
    
    rospy.spin()