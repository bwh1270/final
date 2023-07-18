#ifndef _CONVERTER_CLASS_
#define _CONVERTER_CLASS_

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdint>

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


class Converter
{
private:
	// Voxel Grid Map Information
	uint8_t x_;
	uint8_t y_;
	uint8_t z_;
	uint16_t voxel_grid_map_size_; // x_ * y_ * z_
	int value_array_[3];       // uav don't know = 0, can go = 1, can not go = 2
	uint8_t vicon_bias_x_; 
	uint8_t vicon_bias_y_;  // for origin moving to corner
	uint8_t vicon_bias_z_;

	// Camera FOV Information
	double depth_FoV_VGA_[3];    // H:75, V:62, D:89 [degree]
	double color_FoV_[3];	     // H:69, V:42, D:77 [degree]

	// Fitted Vehicle Position into Voxel Grid Map
	double vicon_xyz_[3];
	double uav_grid_position_[3];    // [meter]
	double uav_grid_orientation_[4]; // [radian]

	// Octomap Data
	//geometry_msgs::PoseArray pcl2xyz_msgs_;
	converter::xyz pcl2xyz_msgs;

	// Subscriber
	ros::Subscriber vicion_pose_sub_;
	ros::Subscriber octomap_pcl_sub_;

	// Publisher
	//ros::Publisher pcl2xyz_pub_;
	ros::Publisher pcl2xyz_pub;
	//ros::Publisher voxel_grid_map_pub_;
	ros::Publisher uav_xyz_pub_;
	ros::Publisher voxel_grid_map_pub_32;

public:
	Converter(ros::NodeHandle *nh);
	void vicon_pose_callback(const nav_msgs::Odometry::ConstPtr &msg);
	void octomap_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
	double fitting_to_grid(double x);
	void octomap_to_grid();
	double deg2rad(double deg);
};


#endif
