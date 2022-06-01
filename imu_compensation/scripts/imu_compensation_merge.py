#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import Imu, PointCloud2
import message_filters


# def lidar_callback(msg):
#     lidar_msg = PointCloud2()
#     lidar_msg = msg

#     pub_lidar.publish(lidar_msg)


# def imu_callback(msg):
  
#     imu_msg = Imu()
    
#     imu_msg.header.seq = msg.header.seq
#     imu_msg.header.stamp = msg.header.stamp
#     imu_msg.header.frame_id = 'Imu_compensated'

#     imu_msg.orientation.w = msg.orientation.w  
#     imu_msg.orientation.x = msg.orientation.x  
#     imu_msg.orientation.y = msg.orientation.y  
#     imu_msg.orientation.z = msg.orientation.z 


#     imu_msg.angular_velocity.x =  msg.angular_velocity.x
#     imu_msg.angular_velocity.y =  -msg.angular_velocity.y
#     imu_msg.angular_velocity.z =  -msg.angular_velocity.z 


#     imu_msg.linear_acceleration.x = msg.linear_acceleration.x 
#     imu_msg.linear_acceleration.y = -msg.linear_acceleration.y 
#     imu_msg.linear_acceleration.z = -msg.linear_acceleration.z 

#     pub_imu.publish(imu_msg)

def sensor_callback(msg_lidar, msg_imu):
    
    lidar_msg = PointCloud2()
    lidar_msg = msg_lidar

    imu_msg = Imu()
    
    imu_msg.header.seq = msg_imu.header.seq
    imu_msg.header.stamp = msg_imu.header.stamp
    imu_msg.header.frame_id = 'Imu_compensated'

    imu_msg.orientation.w = msg_imu.orientation.w  
    imu_msg.orientation.x = msg_imu.orientation.x  
    imu_msg.orientation.y = msg_imu.orientation.y  
    imu_msg.orientation.z = msg_imu.orientation.z 


    imu_msg.angular_velocity.x =  msg_imu.angular_velocity.x
    imu_msg.angular_velocity.y =  -msg_imu.angular_velocity.y
    imu_msg.angular_velocity.z =  -msg_imu.angular_velocity.z 


    imu_msg.linear_acceleration.x = msg_imu.linear_acceleration.x 
    imu_msg.linear_acceleration.y = -msg_imu.linear_acceleration.y 
    imu_msg.linear_acceleration.z = -msg_imu.linear_acceleration.z 


    pub_lidar.publish(lidar_msg)
    pub_imu.publish(imu_msg)
    


if __name__ == '__main__':
    
    rospy.init_node("imu_compensator")
    
    pub_lidar = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=100)
    pub_imu = rospy.Publisher('/imu/imu_compn', Imu, queue_size=100)
    
    sub_lidar = message_filters.Subscriber("/merged_velodyne_points", PointCloud2)
    sub_imu = message_filters.Subscriber("/imu/imu", Imu)
    ts = message_filters.ApproximateTimeSynchronizer([sub_lidar, sub_imu], 100, 0.1, allow_headerless=False)
    ts.registerCallback(sensor_callback)
    
    rospy.spin()