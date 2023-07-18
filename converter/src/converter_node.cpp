#include "ros/ros.h"
#include "converter/converter_class.hpp"
#include "converter/converter_2d_class.hpp"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "converter_node");
	ros::NodeHandle nh;

//Converter converter = Converter(&nh);
	ROS_INFO("Converter Class is activated");
	//Converter2D converter2d = Converter2D(&nh);
	ros::spin();
	return 0;
}