#ifndef _CONVERTER_2D_CLASS_
#define _CONVERTER_2D_CLASS_

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/UInt32MultiArray.h"
#include "nav_msgs/Odometry.h"

#include "converter/orientation.hpp"
#include "converter/xyz.h"

#define CARROT_PI (3.14159265358979)


class Converter2D
{
private:
	// Voxel Grid Map Information
	double x_;
	double y_;
	int voxel_grid_map_size_; // x_ * y_ * z_
	int value_array_[3];       // uav don't know = 0, can go = 1, can not go = 2
	double vicon_bias_x_; 
	double vicon_bias_y_;  // for origin moving to corner
    int line_grid_res_;
	
    // Camera FOV Information
	double depth_FoV_VGA_[3];    // H:75, V:62, D:89 [degree]
	double color_FoV_[3];	     // H:69, V:42, D:77 [degree]

	// Fitted Vehicle Position into Voxel Grid Map
	double vicon_xy_[2];
	double uav_grid_position_[2];    // [meter]
	double uav_grid_orientation_[4]; // [radian]

	// Octomap Data
	converter::xyz pcl2xy_msgs;

	// Subscriber
	ros::Subscriber vicion_pose_sub_;
	ros::Subscriber octomap_pcl_sub_;

	// Publisher
	//ros::Publisher pcl2xyz_pub_;
    ros::Publisher uav_xy_pub_;
	ros::Publisher pcl2xy_pub;
	//ros::Publisher voxel_grid_map_pub_;
	ros::Publisher voxel_grid_map_pub_32;

public:
	Converter2D(ros::NodeHandle *nh);
	void vicon_pose_callback(const nav_msgs::Odometry::ConstPtr &msg);
	void octomap_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
	double fitting_to_grid(double x);
	void octomap_to_grid();
	double deg2rad(double deg);
};


#endif
