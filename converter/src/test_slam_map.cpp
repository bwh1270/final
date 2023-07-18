#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "converter/xyz.h"
#include "std_msgs/UInt32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"


// Global variables for uav_xyz
double uav_x = 9;
double uav_y = 5;
double uav_z = 0;


void uav_xy_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	uav_x = msg->pose.position.x;
	uav_y = msg->pose.position.y;
	uav_z = msg->pose.position.z;
}

int indexing(int x, int y, int step_y)
{
	int ans;
	ans = x * step_y + y;
	return ans;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_slam_map_node");
	ros::NodeHandle nh;

	ros::Publisher slam_map_pub = nh.advertise<std_msgs::UInt32MultiArray>("/carrot_team/voxel_grid_map_32", 10);
	ros::Publisher pcl2xyz_pub = nh.advertise<converter::xyz>("/carrot_team/pcl2xyz", 10);
	ros::Publisher uav_xy_pub = nh.advertise<geometry_msgs::Point>("/carrot_team/uav_xyz", 10);
	ros::Subscriber uav_xy_sub = nh.subscribe("/red/tracker/input_pose", 10, uav_xy_callback);

	/* SLAM MAP */
	int size_x = 43;
	int size_y = 31;
	int total_size = size_x * size_y;

	// can go: 1, can't go: 2
	double slam_map[total_size] = {1, };

	std_msgs::UInt32MultiArray slam_map_msg;
	slam_map_msg.data.resize(total_size);
	
	int i(0), j(0);
	int numOfObs(0);
	for (int k=20; k<26; ++k) {
		for (int l=0; l<3; ++l) {
			slam_map[indexing(14+l, k, size_y)] = 2;
			++numOfObs;
		}
	}
	for (int k=4; k<11; ++k) {
		for (int l=0; l<3; ++l) {
			slam_map[indexing(18+l, k, size_y)] = 2;
			++numOfObs;
		}
	}
	for (int k=0; k<5; ++k) {
		slam_map[indexing(18+k, 10+k, size_y)] = 2;
		slam_map[indexing(18+k, 11+k, size_y)] = 2;
		slam_map[indexing(18+k, 12+k, size_y)] = 2;
		++numOfObs;
	}
	for (int k=21; k<26; ++k) {
		for (int l=0; l<3; ++l) {
			slam_map[indexing(k, 25+l, size_y)] = 2;
			++numOfObs;
		}
	}
	for (int k=0; k<6; ++k) {
		slam_map[indexing(25+k, 25-k, size_y)] = 2;
		slam_map[indexing(25+k, 24-k, size_y)] = 2;
		slam_map[indexing(25+k, 23-k, size_y)] = 2;
		++numOfObs;
	}
	
	for (int m=0; m<size_x; ++m) {
		for (int n=0; n<size_y; ++n) {
			slam_map_msg.data[n+size_y*m] = slam_map[n+size_y*m];
			++numOfObs;
		}
	}
	

	/* PCL2XYZ */
	converter::xyz pcl2xy_msgs;
	pcl2xy_msgs.xyz.resize(numOfObs);
	for (int m=0; m<size_x; ++m) {
		for (int n=0; n<size_y; ++n) {
			if (slam_map[n + size_y * m] == 2) {
				geometry_msgs::Point point_msg;
				point_msg.x = m;
				point_msg.y = n;
				point_msg.z = -1;
				pcl2xy_msgs.xyz.push_back(point_msg);
			}
		}
	}

	/* UAV_XYZ */
	// initial pose
	geometry_msgs::Point uav_xy_msg;
	uav_xy_msg.x = uav_x;
	uav_xy_msg.y = uav_y;
	uav_xy_msg.z = uav_z;

	ros::Rate r(10);
	while (ros::ok())
	{
		slam_map_pub.publish(slam_map_msg);
		pcl2xyz_pub.publish(pcl2xy_msgs);
		uav_xy_pub.publish(uav_xy_msg);
		r.sleep();
	}
	return 0;
}

	
