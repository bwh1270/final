#include "ros/ros.h"
#include "converter/converter_2d_class.hpp"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "converter_2d_node");
	ros::NodeHandle nh;

	ROS_INFO("Converter2D Class is activated");
	Converter2D converter2d = Converter2D(&nh);
	ros::spin();
	return 0;
}

 
