#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>


void LaserScanMsgRecieved(const sensor_msgs::LaserScan &msg){
	int numLas = round((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1;
	double conv = 3.14159/180;
	ROS_INFO_STREAM("The max angle is " << msg.angle_max / conv);
	ROS_INFO_STREAM("The min angle is " << msg.angle_min / conv);
	ROS_INFO_STREAM("The angle increment is " << msg.angle_increment / conv);



	for (int i=0; i < numLas; i++){
		// ROS_INFO_STREAM(msg.ranges.at(i));

		if (msg.ranges.at(i) < 5){
			double obs_angle = (msg.angle_min + msg.angle_increment*i)/conv;
			double radius = msg.ranges.at(i);
			ROS_WARN_STREAM("There is an obstacle at " << obs_angle << 
			"degrees and " << radius << "meters away");
		}

	}
}

int main(int argc, char **argv) {
	// Initialize the ROS system and become a node
	ros::init(argc, argv, "obstacle_avoid");
	ros::NodeHandle nh;

	// Creat a subscriber object
	ros::Subscriber lidar_sub = nh.subscribe("base_scan", 1,
		&LaserScanMsgRecieved);
	// =node_handle.subscribe(topic_name, queue_size, &pointer_to_cb_function)

	// Let ROS take over
	ros::spin();
}