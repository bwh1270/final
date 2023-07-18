#include "converter/vicon_tf_class.hpp"


ViconTF::ViconTF(ros::NodeHandle *nh)
{
	vicon_sub_ = nh->subscribe("/falconblack/vrpn_client/estimated_odometry", 100, &ViconTF::poseCallback, this);
}

void ViconTF::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {

	static tf2_ros::TransformBroadcaster map_br;
	geometry_msgs::TransformStamped map_transformStamped;
	
	map_transformStamped.header.stamp = msg->header.stamp;
	map_transformStamped.header.frame_id = "map";
	map_transformStamped.child_frame_id = "optitrack";
	map_transformStamped.transform.translation.x = msg->pose.pose.position.x;
	map_transformStamped.transform.translation.y = msg->pose.pose.position.y;
	map_transformStamped.transform.translation.z = msg->pose.pose.position.z;
	map_transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
	map_transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
	map_transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
	map_transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
	map_br.sendTransform(map_transformStamped);

	static tf2_ros::TransformBroadcaster vicon_br;
	geometry_msgs::TransformStamped transformStamped;
	
	transformStamped.header.stamp = msg->header.stamp;
	transformStamped.header.frame_id = "optitrack";
	transformStamped.child_frame_id = "falconblack";
	transformStamped.transform.translation.x = msg->pose.pose.position.x;
	transformStamped.transform.translation.y = msg->pose.pose.position.y;
	transformStamped.transform.translation.z = msg->pose.pose.position.z;
	transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
	transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
	transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
	transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
	vicon_br.sendTransform(transformStamped);
	





	/*
	transformStamped.header.stamp = msg->header.stamp; //ros::Time::now();
	transformStamped.header.frame_id = "map"; // or world
	transformStamped.child_frame_id  = "vicon";
	transformStamped.transform.translation.x = msg->pose.pose.position.x;
	transformStamped.transform.translation.y = msg->pose.pose.position.y;
	transformStamped.transform.translation.z = msg->pose.pose.position.z;
	transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
	transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
	transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
	transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
	vicon_br.sendTransform(transformStamped);
	*/
	static tf2_ros::StaticTransformBroadcaster camera_br;
	geometry_msgs::TransformStamped static_transformStamped;
	static_transformStamped.header.stamp = msg->header.stamp; //ros::Time::now();
	static_transformStamped.header.frame_id = "falconblack";
	static_transformStamped.child_frame_id  = "camera_frame";
	static_transformStamped.transform.translation.x = 0.083; // 8.3cm
	static_transformStamped.transform.translation.y = -0.030;
	static_transformStamped.transform.translation.z = -0.045;
	tf2::Quaternion camera_q;
	camera_q.setRPY(-1.570796, 0, -1.570796);  // (0, 15, 0)
	static_transformStamped.transform.rotation.x = camera_q.x();
	static_transformStamped.transform.rotation.y = camera_q.y();
	static_transformStamped.transform.rotation.z = camera_q.z();
	static_transformStamped.transform.rotation.w = camera_q.w();
	camera_br.sendTransform(static_transformStamped);

	static tf2_ros::StaticTransformBroadcaster color_br;
	geometry_msgs::TransformStamped color_transformStamped;
	color_transformStamped.header.stamp = msg->header.stamp; //ros::Time::now();
	color_transformStamped.header.frame_id = "camera_frame";
	color_transformStamped.child_frame_id  = "/red/camera_color_optical_frame";
	color_transformStamped.transform.translation.x = 0;
	color_transformStamped.transform.translation.y = 0;
	color_transformStamped.transform.translation.z = 0;
	tf2::Quaternion color_q;
	color_q.setRPY(-0.261799, 0, 0);
	color_transformStamped.transform.rotation.x = color_q.x();
	color_transformStamped.transform.rotation.y = color_q.y();
	color_transformStamped.transform.rotation.z = color_q.z();
	color_transformStamped.transform.rotation.w = color_q.w();
	color_br.sendTransform(color_transformStamped);

	static tf2_ros::StaticTransformBroadcaster depth_br;
	geometry_msgs::TransformStamped depth_transformStamped;
	depth_transformStamped.header.stamp = msg->header.stamp; //ros::Time::now();
	depth_transformStamped.header.frame_id = "/red/camera_color_optical_frame";
	depth_transformStamped.child_frame_id  = "/red/camera_depth_optical_frame";
	depth_transformStamped.transform.translation.x = 0;
	depth_transformStamped.transform.translation.y = -0.0175;
	depth_transformStamped.transform.translation.z = 0;
	depth_transformStamped.transform.rotation.x = 0;
	depth_transformStamped.transform.rotation.y = 0;
	depth_transformStamped.transform.rotation.z = 0;
	depth_transformStamped.transform.rotation.w = 1;
	depth_br.sendTransform(depth_transformStamped);
}
