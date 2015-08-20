#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/tf.h>
#include <cstdlib>




class EdgeFollower{
	public:
		EdgeFollower(){
			lidar_sub = nh.subscribe("base_scan", 1,
			&EdgeFollower::LaserScanMsgRecieved, this);
			// This is a subscriber object that will subscribe to the robot's odom topic
  			odom_sub = nh.subscribe("odom",1, &EdgeFollower::odom_CB, this);
			//This is the advertising  object that will publish the desired forward and angular velocity of the robot.
  			vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  			ros::Rate rate(10.0);
  			SPEED=2.5;
  			DISTANCE=5;
  			v = 0;
  			w = 0;
		}
		void LaserScanMsgRecieved(const sensor_msgs::LaserScan &g_laser){
  			geometry_msgs::Twist vel;
  			int numLas = round((g_laser.angle_max - g_laser.angle_min) / g_laser.angle_increment) + 1;
			int front = ceil((numLas-1)/2);
			int left = numLas-1;
			int right = 0;

			// Gets new distance needed to find theta

			double dist = g_laser.ranges.at(right);
			ROS_INFO_STREAM("The dist is " << dist);

			// int x= 10;
			// double average=0;

			//for (int i = right; i < x; i++){
				//average = (average + g_laser.ranges.at(i));
			//}

			// average = average/x; 
			double c_range= 30;
			for(int i=right; i<front+1; i++){
				if(g_laser.ranges.at(i)<c_range){
					c_range=g_laser.ranges.at(i);
				}
			}

			int index= 10;
			double phi= index*g_laser.angle_increment;
			double distI= g_laser.ranges.at(index);
			double k= sqrt( (dist*dist)+(distI*distI)-(2*dist*distI*cos(phi)) );
			double theta= asin( ((distI-(dist/cos(phi)))/k)*cos(phi) );
			//ROS_WARN_STREAM("Gamma = " << gamma*180/M_PI);
			//double theta= gamma-M_PI/2;

			// double timestep= 0.1; // timestep in your world file? in seconds
			// ROS_WARN_STREAM("Dist-OldDist = " << dist-oldDist);
			// double theta = atan2(((dist-oldDist)/timestep),(v+((dist+oldDist)/2)*w));
			
			/* if(theta<-M_PI/2){
				theta= theta + M_PI;
			}
			else if (theta > M_PI/2){
				theta= theta -M_PI;
			} */



			ROS_INFO_STREAM("Theta = " << theta*180/M_PI);

			double dMax= 10.0;
			double dMin = 4.9;
			double dFront = g_laser.ranges.at(front);

			// vel.linear.x = SPEED;
			ROS_INFO_STREAM("Front = " << dFront);
			vel.linear.x = std::max(std::min( (((dMax-dMin)/dMax)*SPEED*(dFront-dMin)), SPEED ),0.0);
			vel.angular.z = 3*(DISTANCE-c_range) - 2 * theta;

			ROS_INFO_STREAM("V = " << v);
			ROS_INFO_STREAM("Theta - oldTheta = " << fabs(theta-oldTheta));

			/* if( vel < 0.5){
				hitWall=1;
				ROS_WARN_STREAM("Hit wall!");
			} */

			/* if (v < 1){
				if (fabs(theta-oldTheta)>25.0*M_PI/180){
					vel.linear.x= SPEED;
					ROS_WARN_STREAM("GO!!");
				}
			} */ 

			

			ROS_INFO_STREAM("Angular speed = " << w );
			ROS_INFO_STREAM("Linear speed = " << v );

			// vel.angular.z = 4*(DISTANCE-dist);


			vel_pub.publish(vel);
			// oldDist = dist;
			oldTheta = theta;
		}
		void odom_CB(const nav_msgs::Odometry &msg){
  			//Write published Odometry message to a global variable
  			v = msg.twist.twist.linear.x;
  			w = msg.twist.twist.angular.z;
		}

	private:
		ros::NodeHandle nh;
		ros::Subscriber lidar_sub;
		ros::Subscriber odom_sub;
		ros::Publisher vel_pub;	
		double DISTANCE;
		double SPEED;
		double oldDist;
		double oldTheta;
		double v;
		double w;


};




int main(int argc, char **argv) {
	ros::init(argc, argv, "edge_following");	
	EdgeFollower ef;
	ros::spin();


	return 0;
}