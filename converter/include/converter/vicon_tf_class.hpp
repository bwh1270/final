#ifndef _VICON_TF_CLASS_
#define _VICON_TF_CLASS_

#include <ros/ros.h>

#include <cstdio>
#include <iostream>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "nav_msgs/Odometry.h"


class ViconTF
{
private:
	ros::Subscriber vicon_sub_;

public:
	ViconTF(ros::NodeHandle *nh);
	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);
};



#endif