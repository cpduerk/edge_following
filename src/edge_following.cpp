#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/tf.h>


sensor_msgs::LaserScan g_laser;
nav_msgs::Odometry g_odom;
double g_oldDist;

void odom_CB(const nav_msgs::Odometry &msg){
  //Write published Odometry message to a global variable
  g_odom = msg;
}

void LaserScanMsgRecieved(const sensor_msgs::LaserScan &msg){
	g_laser = msg;
}





int main(int argc, char **argv) {
	ros::init(argc, argv, "edge_following");
	ros::NodeHandle nh;
	// Subscriber to lidar data
	ros::Subscriber lidar_sub = nh.subscribe("base_scan", 1,
		&LaserScanMsgRecieved);
	// This is a subscriber object that will subscribe to the robot's odom topic
  	ros::Subscriber odom_sub = nh.subscribe("odom",1, &odom_CB);
	//This is the advertising  object that will publish the desired forward and angular velocity of the robot.
  	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  	// Initialize the velocity command that we will publish.
  	geometry_msgs::Twist vel;

  	// Desired distance for the robot to be following the wall at
  	double DISTANCE = 5;
  	double SPEED = 2.5;

	
	ros::Rate rate(10.0);
	while ( nh.ok() and (g_laser.ranges.size() == 0) ){
		ros::spinOnce();
		ROS_INFO_STREAM("The size of laser is " << g_laser.ranges.size());
		rate.sleep();
	}

	while ( nh.ok() ){

		ros::spinOnce();
		// ROS_INFO_STREAM("The size of laser is " << g_laser.ranges.size());
		// ROS_WARN_STREAM("Got here");
		// Variables used for referencing lidar data received from CB Function
  		int numLas = round((g_laser.angle_max - g_laser.angle_min) / g_laser.angle_increment) + 1;
		int front = ceil((numLas-1)/2);
		int left = numLas-1;
		int right = 0;
		g_oldDist = g_laser.ranges.at(right);

		// Gets new distance needed to find theta
		ros::spinOnce();

		double dist = g_laser.ranges.at(right);
		ROS_INFO_STREAM("The dist is " << dist);

		int x= 10;
		double average=0;

		for (int i = right; i < x; i++){
			average = (average + g_laser.ranges.at(i));
		}

		average = average/x; 

		double timestep= 0.1; // timestep in your world file? in seconds
		// double theta = atan2( (((dist-g_oldDist)/timestep)*(1.0/(SPEED+((dist+g_oldDist)/2)*vel.angular.z))) )

		vel.linear.x = SPEED;
		vel.angular.z = 3*(DISTANCE-average);

		ROS_INFO_STREAM("Angular speed = " << vel.angular.z);

			// vel.linear.x = SPEED;
			// vel.angular.z = 4*(DISTANCE-dist);


		vel_pub.publish(vel);
		rate.sleep();
	}



	return 0;
}