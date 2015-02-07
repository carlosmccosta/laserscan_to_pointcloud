laserscan_to_pointcloud
=======================

ROS package able to assemble sensor_msgs::LaserScan and publish sensor_msgs::PointCloud2 using spherical linear interpolation (optional and number of TFs to use customizable).
It can publish point clouds after a given number of laser scans has been assemble or at regular intervals (these parameters can be changed dynamically through dynamic_reconfigure or by analyzing nav_msgs::Odometry | sensor_msgs::Imu | geometry_msgs::Twist).
