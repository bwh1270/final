#include "converter/converter_2d_class.hpp"


Converter2D::Converter2D(ros::NodeHandle *nh)
{
	// Voxel Gride Map Information
	x_ = 8.4*5+1; // 0~10 -> 0~20 -> 8.4
	y_ = 6*5+1; // 0~12 -> 0~24 -> 6
	//z_ = 7; // 0~3  -> 0~6
	voxel_grid_map_size_ = (int) (x_ * y_); // 15,488 -> 
	value_array_[0] = 0;
	value_array_[1] = 1;
	value_array_[2] = 2;
	vicon_bias_x_ = 4.2;  // 0~20 mid = 10
	vicon_bias_y_ = 3;  // 0~30 mid = 15
	//vicon_bias_z_ = 0;
    line_grid_res_ = 5*5;

	// Camera FOV Information
	depth_FoV_VGA_[0] = 1.309;
	depth_FoV_VGA_[1] = 1.0821;
	depth_FoV_VGA_[2] = 1.55334;    // H:75, V:62, D:89 [degree]
	color_FoV_[0]     = 1.20428;    // H:69, V:42, D:77 [degree]
	color_FoV_[1]     = 0.733038;
	color_FoV_[2]     = 1.3439;

	// Fitted Vehicle Position into Voxel Grid Map
    vicon_xy_[2] = {0, };
    uav_grid_position_[2] = {0, };
    uav_grid_orientation_[4] = {0, };
    /*
	for (int i=0; i<3; ++i) {
		vicon_xy_[i] = 0;
		uav_grid_position_[i] = 0;       // [meter]
		uav_grid_orientation_[i] = 0;    // [radian]
	}*/

	// Subscriber
	vicion_pose_sub_ = nh->subscribe("/falconblack/vrpn_client/estimated_odometry", 100, &Converter2D::vicon_pose_callback, this);
	octomap_pcl_sub_ = nh->subscribe("/octomap_point_cloud_centers", 100, &Converter2D::octomap_pcl_callback, this);

	// Publisher
	//pcl2xyz_pub_ = nh->advertise<geometry_msgs::PoseArray>("/carrot/pcl2xyz", 10);
	pcl2xy_pub = nh->advertise<converter::xyz>("/carrot_team/pcl2xyz", 10);
	//voxel_grid_map_pub_ = nh->advertise<std_msgs::UInt8MultiArray>("/carrot_team/voxel_grid_map", 50);
	uav_xy_pub_ = nh->advertise<geometry_msgs::Point>("/carrot_team/uav_xyz", 10);
	voxel_grid_map_pub_32 = nh->advertise<std_msgs::UInt32MultiArray>("/carrot_team/voxel_grid_map_32", 10);
}


double Converter2D::fitting_to_grid(double x)
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
		if (two_num < 10) 
			return int_part;

		else if (two_num < 30)
			return int_part + 0.2;
		
		else if (two_num < 50)
			return int_part + 0.4;
		
		else if (two_num < 70)
			return int_part + 0.6;
		
		else if (two_num < 90)
			return int_part + 0.8;
		
		else 
			return int_part + 1;
	}
	else 
	{
		if (two_num > 90)
			return -(int_part + 1);

		else if (two_num > 70)
			return -(int_part + 0.8);

		else if (two_num > 50)
			return -(int_part + 0.6);
		
		else if (two_num > 30)
			return -(int_part + 0.4);

		else if (two_num > 10)
			return -(int_part + 0.2);

		else 
			return -(int_part);
	}  
}


/* @brief Callback Function */
void Converter2D::vicon_pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	double vicon_xy[2];
	//bool vicon_xyz_sign[3] = {true, true, true}; // true: +, false: -

	// Get message values
	vicon_xy[0] = msg->pose.pose.position.x;
	vicon_xy[1] = msg->pose.pose.position.y;
	//vicon_xyz[2] = msg->pose.pose.position.z;
	vicon_xy_[0] = msg->pose.pose.position.x;
	vicon_xy_[1] = msg->pose.pose.position.y;
	//vicon_xyz_[2] = msg->pose.pose.position.z;
	uav_grid_orientation_[0] = msg->pose.pose.orientation.x;
	uav_grid_orientation_[1] = msg->pose.pose.orientation.y;
	uav_grid_orientation_[2] = msg->pose.pose.orientation.z;
	uav_grid_orientation_[3] = msg->pose.pose.orientation.w;

	// fitting vicon xyz to voxel grid map xyz
	for (int i=0; i<2; ++i) {
		uav_grid_position_[i] = Converter2D::fitting_to_grid(vicon_xy[i]);
	}
	geometry_msgs::Point uav_xy_msg;
	uav_xy_msg.x = (uav_grid_position_[0] + vicon_bias_x_) * 2;
	uav_xy_msg.y = (uav_grid_position_[1] + vicon_bias_y_) * 2;
	uav_xy_msg.z = -1;
    //uav_xyz_msg.z = (uav_grid_position_[2] + vicon_bias_z_) * 2;
	//uav_xyz_msg.x = uav_grid_position_[0];
	//uav_xyz_msg.y = uav_grid_position_[1];
	//uav_xyz_msg.z = uav_grid_position_[2];
	uav_xy_pub_.publish(uav_xy_msg);
	//ROS_INFO("uav_grid_position: [%f, %f, %f]", uav_grid_position_[0], uav_grid_position_[1], uav_grid_position_[2]);
}


void Converter2D::octomap_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	double obstacle_xyz[2];
	//bool obstacle_xyz_sign[3] = {true, true, true};

	// Get frame id
	//pcl2xyz_msgs_.header.stamp = msg->header.stamp;
	//pcl2xyz_msgs_.header.frame_id = msg->header.frame_id;
	pcl2xy_msgs.header.stamp = msg->header.stamp;
	pcl2xy_msgs.header.frame_id = msg->header.frame_id;

	// Calculate => # of xyz 
	int pointBytes = msg->point_step;
	int totalBytes = msg->row_step;
	int numOfPoints = (totalBytes / pointBytes);

	// Resize the container
	//pcl2xyz_msgs_.poses.resize(numOfPoints);
	//pcl2xy_msgs.xyz.resize(numOfPoints);
	pcl2xy_msgs.xyz.reserve(numOfPoints);
	//ROS_INFO("[%d]", pcl2xyz_msgs_.poses.size());
	
	// Create point cloud iterator
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

	int cnt = 0;
	for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
	{
		// Access x, y, z values
		//pcl2xyz_msgs_.poses[i].position.x = *iter_x;
		//pcl2xyz_msgs_.poses[i].position.y = *iter_y;
		//pcl2xyz_msgs_.poses[i].position.z = *iter_z;
		obstacle_xyz[0] = *iter_x;
		obstacle_xyz[1] = *iter_y;
		obstacle_xyz[2] = *iter_z;

        if ((obstacle_xyz[2] < 1) || (obstacle_xyz[2] > 2)) {
            // Fitting x,y,z into grid map
            ////pcl2xyz_msgs.xyz[i].x = (Converter::fitting_to_grid(obstacle_xyz[0]) + vicon_bias_x_) * 2;
            ////pcl2xyz_msgs.xyz[i].y = (Converter::fitting_to_grid(obstacle_xyz[1]) + vicon_bias_y_) * 2;
            //pcl2xyz_msgs.xyz[i].z = (Converter::fitting_to_grid(obstacle_xyz[2]) + vicon_bias_z_) * 2;
            ////pcl2xyz_msgs.xyz[i].z = -1;
            geometry_msgs::Point point_msg;
            point_msg.x = (Converter2D::fitting_to_grid(obstacle_xyz[0]) + vicon_bias_x_) * 2;
            point_msg.y = (Converter2D::fitting_to_grid(obstacle_xyz[1]) + vicon_bias_x_) * 2;
            point_msg.z = -1;
            pcl2xy_msgs.xyz.push_back(point_msg);
            ++cnt;
        }
		//pcl2xyz_msgs.xyz[i].x = *iter_x;
		//pcl2xyz_msgs.xyz[i].y = *iter_y;
		//pcl2xyz_msgs.xyz[i].z = *iter_z;
		//ROS_INFO("[%f, %f, %f]", pcl2xyz_msgs.xyz[i].x, pcl2xyz_msgs.xyz[i].y, pcl2xyz_msgs.xyz[i].z);
	}
	//pcl2xy_msgs.xyz.erase(pcl2xy_msgs.xyz.begin() + cnt + 1, pcl2xy_msgs.xyz.end());
    //ROS_INFO("cnt: [%d]", cnt);
	//pcl2xy_msgs.xyz.erase(pcl2xy_msgs.xyz.begin()+ cnt + 1, pcl2xy_msgs.xyz.begin()+numOfPoints);
	//pcl2xyz_pub_.publish(pcl2xyz_msgs_);
	pcl2xy_pub.publish(pcl2xy_msgs);

	Converter2D::octomap_to_grid();

	// initializing
	pcl2xy_msgs.xyz.resize(1);
}


double Converter2D::deg2rad(double deg) {
	double rad = (deg/180) * CARROT_PI;
	return rad;
}


void Converter2D::octomap_to_grid()
{
	int n;
	int N;
	double L;

	// find L
	L = 10 * tan(depth_FoV_VGA_[0] / 2);

	// number of grid in horizontal axis, number of grid in vertical axis
	n = ceil(L);
	N = 2*n + 1;
	
	// find FoV points at 5 meters
	double theta_i;  
	double yaw_i;
	double zeta[N][2] = {0, }; // N points of (x,y) vector
	double points_array[N][2] = {0, };

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
		theta_i = atan2(L*i, 10*n);
		yaw_i = current_yaw + theta_i;
		zeta[i+n][0] = cos(yaw_i) * 5 / cos(theta_i);
		zeta[i+n][1] = sin(yaw_i) * 5 / cos(theta_i);
		//ROS_INFO("zeta: %f", zeta[i+n][0]);
	}

	for (int i=0; i<N; ++i) {
			points_array[i][0] = vicon_xy_[0] + zeta[i][0];
			points_array[i][1] = vicon_xy_[1] + zeta[i][1];
    }
	
	// fitting vicon xyz to voxel grid map xyz
	double points_array_grid[N][2] = {0, };
	
	for (int i = 0; i<N; ++i) {
		for (int j=0; j<2; ++j) {
			points_array_grid[i][j] = Converter2D::fitting_to_grid(points_array[i][j]);
		}
	}

	
	// make a grid in line b/w each point and uav position
	double line_grid_one[2] = {0, 0}; // test
	double fitted_line_grid[N*line_grid_res_][3];
	for (int i=0; i<N*line_grid_res_; ++i) { fitted_line_grid[i][0] = 0; fitted_line_grid[i][1] = 0; fitted_line_grid[i][2] = 0;}

	for (int i=0; i<N; ++i) {
		for (int k=1; k<=line_grid_res_; ++k) {
	
			line_grid_one[0] = vicon_xy_[0] + ((points_array_grid[i][0] - vicon_xy_[0]) * k / line_grid_res_);
			line_grid_one[1] = vicon_xy_[1] + ((points_array_grid[i][1] - vicon_xy_[1]) * k / line_grid_res_);

			fitted_line_grid[k+line_grid_res_*i][0] = Converter2D::fitting_to_grid(line_grid_one[0]);
			fitted_line_grid[k+line_grid_res_*i][1] = Converter2D::fitting_to_grid(line_grid_one[1]);	
		}
	}

		
	// grid to publish
	std_msgs::UInt32MultiArray fov_voxel_grid_map_32_msg;
	fov_voxel_grid_map_32_msg.data.resize(voxel_grid_map_size_);

	// fill the value 0 into voxel grid map msg (default)
	for (int i=0; i<voxel_grid_map_size_; ++i) {
		//fov_voxel_grid_map_msg.data[i] = 0;
		fov_voxel_grid_map_32_msg.data[i] = 0;
	}
	
	int temp_x(0), temp_y(0);
	int temp_idx(0);
	// fill the value 1 into voxel grid map msg
	for (int i=0; i<N*line_grid_res_; ++i) {
		// to 0~21, 0~31, 0~21
		temp_x = (int) ((fitted_line_grid[i][0] + vicon_bias_x_) * 2); 
		temp_y = (int) ((fitted_line_grid[i][1] + vicon_bias_y_) * 2);
		if (((temp_x >= 0) && (temp_x <= x_)) &&
			((temp_y >= 0) && (temp_y <= y_))) {
			temp_idx = (int) (temp_y + y_ * temp_x);
			fov_voxel_grid_map_32_msg.data[temp_idx] = 1;
		}
	}
	
	// fill the value 2 into voxel grid map msg about obstacle detected in octomap
	int size_xy = pcl2xy_msgs.xyz.size();
	for (int i=0; i<size_xy; ++i) {
		if (((pcl2xy_msgs.xyz[i].x >= 0) && (pcl2xy_msgs.xyz[i].x <= x_)) &&
			((pcl2xy_msgs.xyz[i].y >= 0) && (pcl2xy_msgs.xyz[i].y <= y_))) {
			temp_idx = (int) (pcl2xy_msgs.xyz[i].y + y_ * pcl2xy_msgs.xyz[i].x);
			fov_voxel_grid_map_32_msg.data[temp_idx] = 2;
		}
	}
	voxel_grid_map_pub_32.publish(fov_voxel_grid_map_32_msg);
}
