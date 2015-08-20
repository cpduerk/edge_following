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
  			DISTANCE=1;
  			v = 0;
  			w = 0;
  			fsm=4;
  			wasNeg=0;
  			omega_max = 1.5;
  			initiatedOTurn = 0;
  			oldDist = 30;
  			waypointSet = 0;
  			robX = 0;
  			robY = 0;
  			changer = 0;
  			five_secs = ros::Duration(10.0);
  			ROS_WARN_STREAM("FSM = " << fsm);
		}
		void LaserScanMsgRecieved(const sensor_msgs::LaserScan &g_laser){
  			geometry_msgs::Twist vel;
  			int numLas = round((g_laser.angle_max - g_laser.angle_min) / g_laser.angle_increment) + 1;
			int front = ceil((numLas-1)/2);
			int left = numLas-1;
			int right = 0;
			double lidar_to_COR= 0.2; //this is the distance from the lidar to the center or rotation

			// Gets new distance needed to find theta

			double dRight = g_laser.ranges.at(right);
			ROS_INFO_STREAM("The dist is " << dRight);

			// int x= 10;
			// double average=0;

			//for (int i = right; i < x; i++){
				//average = (average + g_laser.ranges.at(i));
			//}

			// average = average/x; 
			double c_range= 30;
			for(int i=right; i<ceil(front/2.0)+1; i++){
				if(g_laser.ranges.at(i)<c_range){
					c_range=g_laser.ranges.at(i);
				}
			}

			double any_range= 30;
			for(int i=right; i<numLas; i++){
				if(g_laser.ranges.at(i)<any_range){
					any_range=g_laser.ranges.at(i);
				}
			}



			int index= 10;
			double phi= index*g_laser.angle_increment;
			double distI= g_laser.ranges.at(index);
			double k= sqrt( (dRight*dRight)+(distI*distI)-(2*dRight*distI*cos(phi)) );
			double theta= asin( ((distI-(dRight/cos(phi)))/k)*cos(phi) );
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

			double dMax= DISTANCE+5;
			double dMin = DISTANCE-lidar_to_COR-0.1;
			double dFront = g_laser.ranges.at(front);

			// vel.linear.x = SPEED;
			ROS_INFO_STREAM("Front = " << dFront);
			ROS_INFO_STREAM("Old dist = " << oldDist); 
			ROS_INFO_STREAM("DistI = " << distI);

			if ( (fsm == 0) and (distI-oldDist>5) ){
				fsm = 2;
				ROS_WARN_STREAM("Stopped- going to outside turn mode");
			}

			if ( (fsm == 0) and (waypointSet == 0) and (fabs(c_range-DISTANCE) < 0.01) and (fabs(theta) < 5*M_PI/180)){
				ROS_WARN_STREAM("Can put way point here");
				waypoint_x = g_odom.pose.pose.position.x;
				waypoint_y = g_odom.pose.pose.position.y;
				waypointSet = 1;
				setWP = ros::Time::now();
				ROS_WARN_STREAM("Way point set");

			}

			if(waypointSet == 1){
				ROS_WARN_STREAM("The way point position is (" << waypoint_x << "," << waypoint_y << ")");
			}

			if( (waypointSet ==1) and (fabs(robX- waypoint_x) < 0.75) and (fabs(robY-waypoint_y) < 0.75) and (ros::Time::now()-setWP > five_secs) ){
				DISTANCE = DISTANCE + 1;
				waypointSet = 0;
				ROS_ERROR_STREAM("Need to find new waypoint");
			}

			ROS_WARN_STREAM("Robot: (" << robX << "," << robY << ")");

			if (fsm == 0){
				vel.linear.x = std::max(std::min( (((dMax-dMin)/dMax)*SPEED*(dFront-dMin)), SPEED ),0.0);
				// vel.angular.z = 3*(DISTANCE-c_range) - 2 * theta;
				vel.angular.z = ((3*(DISTANCE-c_range) - 3 * theta) + vel.angular.z)/2.0;
				// vel.angular.z = 3*(DISTANCE-c_range) - 3 * theta;



				if ((dFront <= DISTANCE-lidar_to_COR) /* and (fabs(theta*180/M_PI) < 1.5) */ ){
					fsm = 1;
					ROS_WARN_STREAM("Switched to Rotate");
				}

				/* if ((distI-oldDist)>20){
					fsm = 2;
					ROS_WARN_STREAM("Stopped- going to outside turn mode");
				} */
			}

			else if (fsm==1){
				vel.linear.x = 0;
				// vel.angular.z = 3*(DISTANCE-c_range) - 2 * theta;
				vel.angular.z = 0.5;

				if (theta < -M_PI/8){
					wasNeg = 1;
					ROS_WARN_STREAM("WasNeg is 1");
				}

				if (wasNeg == 1) {
					vel.angular.z = - 0.5 * theta;
				}

				if ((wasNeg == 1) and (fabs(theta) < 0.03)){
					wasNeg = 0;
					fsm = 0;
					ROS_WARN_STREAM("Switched to Wall Following");
				}
			}
			else if (fsm == 2){
				vel.linear.x = 0.5;
				vel.angular.z = 0;

				if (dRight > DISTANCE + 0.5){
					initiatedOTurn= 1;
				}

				if (initiatedOTurn == 1){
					vel.linear.x = 2.5;
					vel.angular.z = -2.5/DISTANCE;

					if (fabs(vel.angular.z) > omega_max ){
						vel.angular.z = -omega_max;
						vel.linear.x = -DISTANCE*vel.angular.z;
					}

					ROS_WARN_STREAM("Stage 1- turning");
				}
				if ((fabs(theta) < 0.1) and (distI<10) and (dRight<10)){
					fsm = 0;
					initiatedOTurn = 0;
					ROS_WARN_STREAM("Switched to Wall Following");
				}

				/* if (dRight >= DISTANCE+1){

					vel.linear.x=1;
					vel.angular.z = -10;
				} */

			}

			else if( fsm == 4){
				vel.linear.x = 1+changer;
				vel.angular.z = M_PI/2;
				if (any_range <= DISTANCE + 0.5){
					fsm = 0;
				}

			}
			ROS_INFO_STREAM("V = " << v);
			// ROS_INFO_STREAM("Theta - oldTheta = " << fabs(theta-oldTheta));

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
			ROS_WARN_STREAM("FSM = " << fsm);

			// vel.angular.z = 4*(DISTANCE-dist);


			vel_pub.publish(vel);
			oldDist = distI;
			oldTheta = theta;
			changer = changer + 0.05;
		}
		void odom_CB(const nav_msgs::Odometry &msg){
  			//Write published Odometry message to a global variable
  			g_odom = msg;
  			robX = msg.pose.pose.position.x;
  			robY = msg.pose.pose.position.y;
  			v = msg.twist.twist.linear.x;
  			w = msg.twist.twist.angular.z;
		}

	private:
		ros::NodeHandle nh;
		ros::Subscriber lidar_sub;
		ros::Subscriber odom_sub;
		ros::Publisher vel_pub;	
		nav_msgs::Odometry g_odom;

		// A little confused where to create/declare times
		ros::Time setWP;
		ros::Duration five_secs;

		double DISTANCE;
		double SPEED;
		double oldDist;
		double oldTheta;
		double v;
		double w;
		int fsm;
		int wasNeg;
		double omega_max;
		int initiatedOTurn;
		int waypointSet;
		double waypoint_x;
		double waypoint_y;
		double robX;
		double robY;
		double changer;



};




int main(int argc, char **argv) {
	ros::init(argc, argv, "edge_following");	
	EdgeFollower ef;
	ros::spin();


	return 0;
}