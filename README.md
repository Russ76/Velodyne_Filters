# Velodyne_Filters

## A pair of filters for indoor and outdoor use

We bought a couple older Velodyne VLP16 Lidar units from eBay. The prices weren't high, nothing like they cost when they were new! It could be, though, that the bearings are worn and the main lidar shaft is slightly wobbly. This can do bad things to lidar points, when the processing computer figures that some points must have come from very far away!  If your robot is using the lidar pointcloud for ICP odometry, these stray points can make the program stumble and fail.

Because of this trouble, I consulted Google and we came up with a couple of C++ processing programs that allow the ICP odometry and pointcloud mapping to continue on successfully. One listing is for indoors, and uses the voxel downsampling function. It changed things for Rtabmap and Kiss-icp to be able to use the pointcloud and successfully track the robot and make maps. This node is called pcl_filter. If you look at the velodyne_points in Rviz you will see that they are dense and numerous. This filter publishes processed_points and they are sparser, easier to compute and easier to send wirelessly. It works very well indoors!

The other program is called lidar filter package and is a dual filter node. It has a pass-through filter and then a statistical outlier removal filter. It is for outdoors. It finds those pesky outlier points and removes them! The filter language comes from PCL, the piont cloud library. It is harder to make these filters than I imagined; we had to try several times to get them right. This filter also subscribes to velodyne_points and it publishes to velodyne_points_filtered. It works very well outdoors!

These programs were tested with Ubuntu 24.04 and Ros2 Jazzy. I used both a Raspberry Pi 5 and a mini-computer with Ryzen 7 processor in the robot. Both worked well.

Hobbyist builders shouldn't be afraid of investing in these used Velodyne Lidar units. Your Lidar will be able to see 100 Meters instead of 12, and across a field of view of 30 degrees vertically, instead of just one line of laserscan. Just add these filters to your arsenal and your icp odometry and pointcloud mapping will sail along, inside or outdoors.

If you need instructions for making a cable to attach the Velodyne VLP16 to your robot, contact me and I'll share the notes I made. It's not hard to make a suitable cable. And each version of ROS has the software to publish the pointcloud and/or just the laserscan, whichever you want.

## Commands:

Inside workspace folder: git clone www.github.com/russ76/velodyne_filter.git

colcon build

source install/setup.bash

ros2 run pcl_filter pcl_filter

ros2 run lidar_filter_package lidar_filter_node
