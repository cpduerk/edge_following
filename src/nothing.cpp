#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

void LaserScanMsgRecieved(const sensor_msgs::LaserScan &msg){
	int front = round((msg.angle_max - msg.angle_min)/msg.angle_increment)/2;


}





int main(int argc, char **argv) {
	// Initialize the ROS system and become a node
	ros::init(argc, argv, "back_and_forth");
	ros::NodeHandle nh;

	// Creat a subscriber object
	ros::Subscriber lidar_sub = nh.subscribe("base_scan", 1,
		&LaserScanMsgRecieved);
	// =node_handle.subscribe(topic_name, queue_size, &pointer_to_cb_function)

	// Creat a subscriber object
	ros::Subscriber odom_sub = nh.subscribe("odom", 1,
		&OdomMsgRecieved);
	// =node_handle.subscribe(topic_name, queue_size, &pointer_to_cb_function)

	//This is the advertising  object that will publish the desired forward and angular velocity of the robot.
  	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

	// Let ROS take over
	ros::spin();
}