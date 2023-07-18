#include "converter/converter_class.hpp"


Converter::Converter(ros::NodeHandle *nh)
{
	// Voxel Gride Map Information
	x_ = 21; // 0~10 -> 0~20
	y_ = 24; // 0~12 -> 0~24
	z_ = 7; // 0~3  -> 0~6:
	voxel_grid_map_size_ = (x_ * y_ * z_); // 15,488 -> 
	value_array_[0] = 0;
	value_array_[1] = 1;
	value_array_[2] = 2;
	vicon_bias_x_ = 5;  // 0~20 mid = 10
	vicon_bias_y_ = 6;  // 0~30 mid = 15
	vicon_bias_z_ = 0;

	// Camera FOV Information
	depth_FoV_VGA_[0] = 1.309;
	depth_FoV_VGA_[1] = 1.0821;
	depth_FoV_VGA_[2] = 1.55334;    // H:75, V:62, D:89 [degree]
	color_FoV_[0]     = 1.20428;    // H:69, V:42, D:77 [degree]
	color_FoV_[1]     = 0.733038;
	color_FoV_[2]     = 1.3439;

	// Fitted Vehicle Position into Voxel Grid Map
	for (int i=0; i<3; ++i) {
		vicon_xyz_[i] = 0;
		uav_grid_position_[i] = 0;       // [meter]
		uav_grid_orientation_[i] = 0;    // [radian]
	}

	// Subscriber
	vicion_pose_sub_ = nh->subscribe("/falconblack/vrpn_client/estimated_odometry", 100, &Converter::vicon_pose_callback, this);
	octomap_pcl_sub_ = nh->subscribe("/octomap_point_cloud_centers", 100, &Converter::octomap_pcl_callback, this);

	// Publisher
	//pcl2xyz_pub_ = nh->advertise<geometry_msgs::PoseArray>("/carrot/pcl2xyz", 10);
	pcl2xyz_pub = nh->advertise<converter::xyz>("/carrot_team/pcl2xyz", 10);
	//voxel_grid_map_pub_ = nh->advertise<std_msgs::UInt8MultiArray>("/carrot_team/voxel_grid_map", 50);
	uav_xyz_pub_ = nh->advertise<geometry_msgs::Point>("/carrot_team/uav_xyz", 10);
	voxel_grid_map_pub_32 = nh->advertise<std_msgs::UInt32MultiArray>("/carrot_team/voxel_grid_map_32", 10);
}


double Converter::fitting_to_grid(double x)
{
	double rounded;
	double int_part;
	double fractional_part;
	uint8_t two_num; // 1.794 -> 79
	bool sign = true;

	if ((x)<0) {
		//*sign = false;
		sign = false;
		x = std::fabs(x);
	}

	rounded = std::round((x) * 100) / 100;
	fractional_part = std::modf(rounded, &int_part);
	two_num = (int) std::round(fractional_part * 100);

	if (sign)
	{
		if (two_num < 25) 
			return int_part;
		
		else if (two_num <= 75) 
			return int_part + 0.5;
		
		else 
			return int_part + 1;
	}
	else 
	{
		if (two_num > 75)
			return -(int_part + 1);

		else if (two_num > 25)
			return -(int_part + 0.5);

		else 
			return -(int_part);
	}  
}


/* @brief Callback Function */
void Converter::vicon_pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	double vicon_xyz[3];
	//bool vicon_xyz_sign[3] = {true, true, true}; // true: +, false: -

	// Get message values
	vicon_xyz[0] = msg->pose.pose.position.x;
	vicon_xyz[1] = msg->pose.pose.position.y;
	vicon_xyz[2] = msg->pose.pose.position.z;
	vicon_xyz_[0] = msg->pose.pose.position.x;
	vicon_xyz_[1] = msg->pose.pose.position.y;
	vicon_xyz_[2] = msg->pose.pose.position.z;
	uav_grid_orientation_[0] = msg->pose.pose.orientation.x;
	uav_grid_orientation_[1] = msg->pose.pose.orientation.y;
	uav_grid_orientation_[2] = msg->pose.pose.orientation.z;
	uav_grid_orientation_[3] = msg->pose.pose.orientation.w;

	// fitting vicon xyz to voxel grid map xyz
	for (int i=0; i<3; ++i) {
		uav_grid_position_[i] = Converter::fitting_to_grid(vicon_xyz[i]);
	}
	geometry_msgs::Point uav_xyz_msg;
	uav_xyz_msg.x = (uav_grid_position_[0] + vicon_bias_x_) * 2;
	uav_xyz_msg.y = (uav_grid_position_[1] + vicon_bias_y_) * 2;
	uav_xyz_msg.z = (uav_grid_position_[2] + vicon_bias_z_) * 2;
	//uav_xyz_msg.x = uav_grid_position_[0];
	//uav_xyz_msg.y = uav_grid_position_[1];
	//uav_xyz_msg.z = uav_grid_position_[2];
	uav_xyz_pub_.publish(uav_xyz_msg);
	//ROS_INFO("uav_grid_position: [%f, %f, %f]", uav_grid_position_[0], uav_grid_position_[1], uav_grid_position_[2]);
}


void Converter::octomap_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	double obstacle_xyz[3];
	//bool obstacle_xyz_sign[3] = {true, true, true};

	// Get frame id
	//pcl2xyz_msgs_.header.stamp = msg->header.stamp;
	//pcl2xyz_msgs_.header.frame_id = msg->header.frame_id;
	pcl2xyz_msgs.header.stamp = msg->header.stamp;
	pcl2xyz_msgs.header.frame_id = msg->header.frame_id;

	// Calculate => # of xyz 
	int pointBytes = msg->point_step;
	int totalBytes = msg->row_step;
	int numOfPoints = (totalBytes / pointBytes);

	// Resize the container
	//pcl2xyz_msgs_.poses.resize(numOfPoints);
	pcl2xyz_msgs.xyz.resize(numOfPoints);
	//ROS_INFO("[%d]", pcl2xyz_msgs_.poses.size());
	
	// Create point cloud iterator
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

	int i = 0;
	for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
	{
		// Access x, y, z values
		//pcl2xyz_msgs_.poses[i].position.x = *iter_x;
		//pcl2xyz_msgs_.poses[i].position.y = *iter_y;
		//pcl2xyz_msgs_.poses[i].position.z = *iter_z;
		obstacle_xyz[0] = *iter_x;
		obstacle_xyz[1] = *iter_y;
		obstacle_xyz[2] = *iter_z;

		// Fitting x,y,z into grid map
		pcl2xyz_msgs.xyz[i].x = (Converter::fitting_to_grid(obstacle_xyz[0]) + vicon_bias_x_) * 2;
		pcl2xyz_msgs.xyz[i].y = (Converter::fitting_to_grid(obstacle_xyz[1]) + vicon_bias_y_) * 2;
		pcl2xyz_msgs.xyz[i].z = (Converter::fitting_to_grid(obstacle_xyz[2]) + vicon_bias_z_) * 2;

		//pcl2xyz_msgs.xyz[i].x = *iter_x;
		//pcl2xyz_msgs.xyz[i].y = *iter_y;
		//pcl2xyz_msgs.xyz[i].z = *iter_z;
		
		++i;
		//ROS_INFO("[%f, %f, %f]", pcl2xyz_msgs.xyz[i].x, pcl2xyz_msgs.xyz[i].y, pcl2xyz_msgs.xyz[i].z);
	}
	//pcl2xyz_pub_.publish(pcl2xyz_msgs_);
	pcl2xyz_pub.publish(pcl2xyz_msgs);

	Converter::octomap_to_grid();

	// initializing
	pcl2xyz_msgs.xyz.resize(1);
}


double Converter::deg2rad(double deg) {
	double rad = (deg/180) * CARROT_PI;
	return rad;
}


void Converter::octomap_to_grid()
{
	int n, m;
	int N, M;
	double L, H;

	// find L, H
	//L = 10 * tan(Converter::deg2rad(depth_FoV_VGA_[0] / 2));
	//H = 10 * tan(Converter::deg2rad(depth_FoV_VGA_[1] / 2));
	L = 10 * tan(depth_FoV_VGA_[0] / 2);
	H = 10 * tan(depth_FoV_VGA_[1] / 2);

	// number of grid in horizontal axis, number of grid in vertical axis
	n = ceil(L);
	m = ceil(H);
	N = 2*n + 1;
	M = 2*m + 1;
	
	//ROS_INFO("[n, m, N, M, L, H] = [%d, %d, %d, %d, %f, %f]", n, m, N, M, L, H);	
	
	// find FoV points at 5 meters
	double theta_i;  
	double z_array[M] = {0, };
	double yaw_i;
	double zeta[N][2] = {0, }; // N points of (x,y) vector
	double points_array[N*M][3] = {0, };
	
	//converter::xyz points_msg;
	//points_msg.xyz.resize(N*M);

	// get current yaw
	double current_yaw;
	Quaternion current_q;
	EulerAngles current_e;
	current_q.x = uav_grid_orientation_[0];
	current_q.y = uav_grid_orientation_[1];
	current_q.z = uav_grid_orientation_[2];
	current_q.w = uav_grid_orientation_[3];
	current_e = ToEulerAngles(current_q);
	current_yaw = current_e.yaw;

	for (int i = -n; i <= n; ++i) {
		theta_i = atan2(L*i, 10*n); // here wrong!!!!
		yaw_i = current_yaw + theta_i;
		zeta[i+n][0] = cos(yaw_i) * 5 / cos(theta_i);
		zeta[i+n][1] = sin(yaw_i) * 5 / cos(theta_i);
		//ROS_INFO("zeta: %f", zeta[i+n][0]);
	}

	for (int j = -m; j <= m; ++j) {
		z_array[j+m] = (H*j) / (2*m); // (H/2) * (j/k)
	}
	for (int i=0; i<N; ++i) {
		//points_array[cnt][0] = vicon_xyz_[0] + zeta[i][0];
		//points_array[cnt][1] = vicon_xyz_[1] + zeta[i][1];
		for (int j=0; j<M; ++j) {
			points_array[j+M*i][0] = vicon_xyz_[0] + zeta[i][0];
			points_array[j+M*i][1] = vicon_xyz_[1] + zeta[i][1];
			points_array[j+M*i][2] = vicon_xyz_[2] + z_array[j];
			//points_msg.xyz[cnt].x = vicon_xyz_[0] + zeta[i][0];
			//points_msg.xyz[cnt].y = vicon_xyz_[1] + zeta[i][1];
			//points_msg.xyz[cnt].z = vicon_xyz_[2] + z_array[j];
		}
	}
	
	// fitting vicon xyz to voxel grid map xyz
	//cnt = 0;
	double points_array_grid[N*M][3] = {0, };
	//bool points_array_sign[N*M][3] = {0, };
	
	for (int i = 0; i<N*M; ++i) {
		for (int j=0; j<3; ++j) {
			//points_array_sign[cnt][j] = true;
			points_array_grid[i][j] = Converter::fitting_to_grid(points_array[i][j]);
			//++cnt;
		}
	}

	
	// make a grid in line b/w each point and uav position  
	//double line_grid[N*M*10][3] = {0, };
	//bool line_grid_sign[N*M*10][3] = {true, true, true};
	double line_grid_one[3] = {0, 0, 0}; // test
	//bool line_grid_one_sign[3] = {true, true, true};     // test
	//double fitted_line_grid[N*M*10][3] = {0, };
	double fitted_line_grid[N*M*10][3];
	for (int i=0; i<N*M*10; ++i) { fitted_line_grid[i][0] = 0; fitted_line_grid[i][1] = 0; fitted_line_grid[i][2] = 0;}
	int res = 10;

	//cnt = 0;
	for (int i=0; i<N*M; ++i) {
		for (int k=1; k<=res; ++k) {
			//line_grid[cnt][0] = vicon_xyz_[0] + ((points_array_grid[i][0] - vicon_xyz_[0]) * k / res);
			//line_grid[cnt][1] = vicon_xyz_[1] + ((points_array_grid[i][1] - vicon_xyz_[1]) * k / res);
			//line_grid[cnt][2] = vicon_xyz_[2] + ((points_array_grid[i][2] - vicon_xyz_[2]) * k / res);
			//line_grid_sign[cnt][0] = true;
			//line_grid_sign[cnt][1] = true;
			//line_grid_sign[cnt][2] = true;
			//fitted_line_grid[cnt][0] = Converter::fitting_to_grid(&line_grid[cnt][0], &line_grid_sign[cnt][0]);
			//fitted_line_grid[cnt][1] = Converter::fitting_to_grid(&line_grid[cnt][1], &line_grid_sign[cnt][1]);
			//fitted_line_grid[cnt][2] = Converter::fitting_to_grid(&line_grid[cnt][2], &line_grid_sign[cnt][2]);
			
			line_grid_one[0] = vicon_xyz_[0] + ((points_array_grid[i][0] - vicon_xyz_[0]) * k / res);
			line_grid_one[1] = vicon_xyz_[1] + ((points_array_grid[i][1] - vicon_xyz_[1]) * k / res);
			line_grid_one[2] = vicon_xyz_[2] + ((points_array_grid[i][2] - vicon_xyz_[2]) * k / res);

			fitted_line_grid[k+res*i][0] = Converter::fitting_to_grid(line_grid_one[0]);
			fitted_line_grid[k+res*i][1] = Converter::fitting_to_grid(line_grid_one[1]);
			fitted_line_grid[k+res*i][2] = Converter::fitting_to_grid(line_grid_one[2]);
			
			//++cnt;
			//std::cout << line_grid_one[0] << std::endl;
			//std::cout << fitted_line_grid[cnt][0] << std::endl;
		}
	}

	/*
	// grid to publish
	//std_msgs::UInt8MultiArray fov_voxel_grid_map_msg;
	//fov_voxel_grid_map_msg.data.resize(voxel_grid_map_size_);

	std_msgs::UInt32MultiArray fov_voxel_grid_map_32_msg;
	fov_voxel_grid_map_32_msg.data.resize(voxel_grid_map_size_);

	// fill the value 0 into voxel grid map msg (default)
	for (int i=0; i<voxel_grid_map_size_; ++i) {
		//fov_voxel_grid_map_msg.data[i] = 0;
		fov_voxel_grid_map_32_msg.data[i] = 0;
	}
	
	int temp_x(0), temp_y(0), temp_z(0);
	int temp_idx(0);
	//int testtt = 0;
	// fill the value 1 into voxel grid map msg
	for (int i=0; i<N*M*10; ++i) {
		// to 0~21, 0~31, 0~21
		temp_x = (int) ((fitted_line_grid[i][0] + vicon_bias_x_) * 2); 
		temp_y = (int) ((fitted_line_grid[i][1] + vicon_bias_y_) * 2);
		temp_z = (int) ((fitted_line_grid[i][2] + vicon_bias_z_) * 2);
		//ROS_INFO("[x, y, z] = [%d, %d, %d]", temp_x, temp_y, temp_z);
		if (((temp_x >= 0) && (temp_x <= x_)) &&
			((temp_y >= 0) && (temp_y <= y_)) &&
			((temp_z >= 0) && (temp_z <= z_))) {
			temp_idx = (int: (temp_z + z_ * (temp_y + y_ * temp_x));
			//fov_voxel_grid_map_msg.data[temp_idx] = 1;
			fov_voxel_grid_map_32_msg.data[temp_idx] = 1;
			//++testtt;
		}
	}
	//ROS_INFO("%d", testtt);

	// fill the value 2 into voxel grid map msg about obstacle detected in octomap
	int size_xyz = pcl2xyz_msgs.xyz.size();
	for (int i=0; i<size_xyz; ++i) {
		if (((pcl2xyz_msgs.xyz[i].x >= 0) && (pcl2xyz_msgs.xyz[i].x <= x_)) &&
			((pcl2xyz_msgs.xyz[i].y >= 0) && (pcl2xyz_msgs.xyz[i].y <= y_)) &&
			((pcl2xyz_msgs.xyz[i].z >= 0) && (pcl2xyz_msgs.xyz[i].z <= z_))) {
			temp_idx = (int) (pcl2xyz_msgs.xyz[i].z + z_ * (pcl2xyz_msgs.xyz[i].y + y_ * pcl2xyz_msgs.xyz[i].x));
			//fov_voxel_grid_map_msg.data[temp_idx] = 2;
			fov_voxel_grid_map_32_msg.data[temp_idx] = 2;
		}
	}
	//ROS_INFO("%d", fov_voxel_grid_map_msg.data.size());
	//voxel_grid_map_pub_.publish(fov_voxel_grid_map_msg);
	voxel_grid_map_pub_32.publish(fov_voxel_grid_map_32_msg);
	*/
}
