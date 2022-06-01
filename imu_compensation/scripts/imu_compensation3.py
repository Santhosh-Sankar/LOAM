#!/usr/bin/env python3

from dataclasses import fields
import rospy 
from sensor_msgs.msg import Imu, PointCloud2
import transforms3d as tf
import numpy as np

# global g_imu_msg
# global g_lidar_msg
# g_imu_msg = None
# g_lidar_msg = None 


# def imu_publisher(event):
#     pub_imu.publish(g_imu_msg)


# def lidar_publisher(event):
#     pub_lidar.publish(g_lidar_msg)


def lidar_callback(msg):
    # global g_lidar_msg
    # g_lidar_msg = msg
    pub_lidar.publish(msg)


def imu_callback(msg):
    # global g_imu_msg
  
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w

    orient = tf.quaternions.quat2mat([w,x,y,z])
    R = np.array([[1,0,0], [0, 1, 0], [0, 0, 1]])
    transform_rot_mat = np.matmul(orient, R)
    transform_quat = tf.quaternions.mat2quat(transform_rot_mat)

    imu_msg = Imu()
    
    imu_msg.header.seq = msg.header.seq
    imu_msg.header.stamp = msg.header.stamp
    imu_msg.header.frame_id = 'Imu_compensated'

    imu_msg.orientation.w = transform_quat[0]
    imu_msg.orientation.x = transform_quat[1]
    imu_msg.orientation.y = transform_quat[2]
    imu_msg.orientation.z = transform_quat[3]

    imu_msg.angular_velocity.x = msg.angular_velocity.x #- 0.0101
    imu_msg.angular_velocity.y =  msg.angular_velocity.y #- 0.0214
    imu_msg.angular_velocity.z =  msg.angular_velocity.z #- 0.0349


    imu_msg.linear_acceleration.x = msg.linear_acceleration.x #- 0.0541
    imu_msg.linear_acceleration.y = msg.linear_acceleration.y #+ 0.2048 
    imu_msg.linear_acceleration.z = msg.linear_acceleration.z # - 0.0607


    pub_imu.publish(imu_msg)
    # g_imu_msg = imu_msg
    
    

if __name__ == '__main__':
    rospy.init_node("imu_compensator")

    pub_imu= rospy.Publisher('/imu/imu_compn', Imu, queue_size=100)
    pub_lidar = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)

    rospy.Subscriber("/imu/imu", Imu, imu_callback)
    rospy.Subscriber('/ns1/velodyne_points', PointCloud2, lidar_callback)   
    # rospy.Timer(rospy.Duration(1.0/10), lidar_publisher) 
    # rospy.Timer(rospy.Duration(1.0/100), imu_publisher)

    rospy.spin()