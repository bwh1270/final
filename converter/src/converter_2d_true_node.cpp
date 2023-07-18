#include "ros/ros.h"
#include "converter/converter_2d_true_class.hpp"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "converter_2d_true_node");
	ros::NodeHandle nh;

//Converter converter = Converter(&nh);
	ROS_INFO("Converter2DTrue Class is activated");
	Converter2DTrue converter2d_true = Converter2DTrue(&nh);
	ros::spin();
	return 0;
}

 
