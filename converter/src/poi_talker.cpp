#include "ros/ros.h"
#include "converter/xyz.h"
#include "icuas23_competition/poi.h"

ros::Publisher poi_pub;

void poi_sub_callback(const icuas23_competition::poi::ConstPtr &msg) {
	converter::xyz poi_arr;
	int poi_len = msg->poi.size();
	poi_arr.xyz.resize(poi_len);

	for (int i=0; i<poi_len; ++i) {
		poi_arr.xyz[i].x = msg->poi[i].x;
		poi_arr.xyz[i].y = msg->poi[i].y;
		poi_arr.xyz[i].z = msg->poi[i].z;
	}

	poi_pub.publish(poi_arr);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "poi_talker");
	ros::NodeHandle nh;

	bool should_latch = true;
	poi_pub = nh.advertise<converter::xyz>("/carrot_team/poi", 10, should_latch);

	ros::Subscriber poi_sub = nh.subscribe("/red/poi", 10, &poi_sub_callback);
	ros::spin();
}

