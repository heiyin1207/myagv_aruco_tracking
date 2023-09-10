# myagv_aruco_tracking

A prototype system built on ROS1 to perform aruco marker tracking. <br/>

On the AGV: <br/>
roslaunch myagv_odometry myagv_activate.launch  <br/>

On the remote machine: <br/>
1/ Download the code into your catkin workspace <br/>
2/ Install the Intel RealSense SDK: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md <br/>
3/ Install librealsense: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python <br/>
4/ Connect the Intel Realsense Camera <br/> 
5/ Build and source the catkin workspace <br/>
6/ Run initAGV.sh <br/>
7/ Run the node with the folowing command <br/> 
```
ros run myagv_aruco_tracking main
```




