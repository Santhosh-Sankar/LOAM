# LiDAR Odometry And Mapping (LOAM)

This repository uses LeGO-LOAM and SC LeGO-LOAM to demonstrate LiDAR Odometry And Mapping (LOAM) along with an IMU compensation package to compensate for the offset in IMU orientation with respect to LiDAR sensor.

Both the LeGO-LOAM and SC LeGO-LOAM were forked and modified to support ROS Noetic and latest GTSAM library.

The data was collected from a pair of Velodyne VLP-16 and Ouster OS1-16 LiDAR sensors and VectorNAV VN-100 IMU mounted on a 2016 Linclon MKZ car.

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (Noetic)
- [gtsam](https://github.com/borglab/gtsam.git) (Georgia Tech Smoothing and Mapping library, 4.1.1)


## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/Santhosh-Sankar/LOAM.git 
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.


## Run the package

1. Delete the LeGO-LOAM or SC LeGO-LOAM pakage which is not to be used.
2. Run the launch file:
```
roslaunch lego_loam run.launch 
```
Notes: The parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.

3. Play existing bag files:
```
rosbag play *.bag --clock --topic /ns1/velodyne_points /imu/imu


