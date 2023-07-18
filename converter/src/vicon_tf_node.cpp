#include "ros/ros.h"
#include "converter/vicon_tf_class.hpp"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_tf_node");
	ros::NodeHandle nh;

	ViconTF vicon_tf = ViconTF(&nh);
	ROS_INFO("Vicon TF class is activated");
	ros::spin();
	return 0;
}