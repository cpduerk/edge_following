#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/tf.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <cmath>


double g_change= 0;
int g_switched = 0;

class RandomWalk {
public:
	// Construct a new RandomWalk object and hook up this ROS node to the 
	// simulated robot's velocity control and laser topics 
	RandomWalk(ros::NodeHandle& nh):
	fsm(FSM_HEADING_1),
	rotateStartTime(ros::Time::now()),
	rotateDuration(0.f){
		// Initialize random time generator
		srand(time(NULL));

		// Advertise a new publisher for the simulated robot's velocity
		commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		// Subscribe to the simulate robot's laser scan topic and tell ROS
		// to call commandCallback() whenever a new message is published on
		// that topic 
		laserSub = nh.subscribe("base_scan", 1, &RandomWalk::commandCallback,
			this);
	};

	// Send a velocity command
	void move(double linearVelMPS, double angularVelRadPS) {
		geometry_msgs::Twist msg;
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);	
	};

	// Process the incoming laser scan message
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		if ((fsm == FSM_HEADING_1)) {
			int numLas = round((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;
			int front = ceil((numLas-1)/2);
			int left = numLas-1;
			int right = 0;

			if ( (msg->ranges.at(front) < PROXIMITY_RANGE_M) and (msg->ranges.at(right) < PROXIMITY_RANGE_M) and (msg->ranges.at(left) >= PROXIMITY_RANGE_M) ) {
				fsm = FSM_ROTATE_2;	
				ROS_INFO_STREAM("FSM = Rotate 2");	
				ROS_INFO_STREAM("Turning counter-clockwise- front and right");
				g_switched++;
				ROS_WARN_STREAM("Switched = " << g_switched);

				if ( (g_switched != 0) and (g_switched % 2 == 0) ){
					g_change = g_change +  (2*M_PI - 3*M_PI/4);
					ROS_WARN_STREAM("Change added");
				}
				// g_change= g_change + (2*M_PI - 3*M_PI/4);
				// ROS_WARN_STREAM("Changed added");
			}
			// Default path going from Heading 1 to Turn 1
			else if (msg->ranges.at(front) < PROXIMITY_RANGE_M) {
				fsm = FSM_ROTATE_1;
				ROS_INFO_STREAM("FSM = Rotate 1");	
				ROS_INFO_STREAM("Clockwise");	
			};
		}; 
		if ((fsm == FSM_HEADING_2)) {
			int numLas = round((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;
			int front = ceil((numLas-1)/2);
			int left = numLas-1;
			int right = 0;


			if ( (msg->ranges.at(front) < PROXIMITY_RANGE_M) and (msg->ranges.at(left) < PROXIMITY_RANGE_M) and (msg->ranges.at(right) >= PROXIMITY_RANGE_M) ) {
				fsm = FSM_ROTATE_1;	
				ROS_INFO_STREAM("FSM = Rotate 1");
				ROS_INFO_STREAM("Turning clockwise- front and left");
				g_switched++;
				ROS_WARN_STREAM("Switched = " << g_switched);

				if ( (g_switched != 0) and (g_switched % 2 == 0) ){
					g_change = g_change +  (2*M_PI - 3*M_PI/4);
					ROS_WARN_STREAM("Change added");
				}
				// g_change = g_change +  (2*M_PI - 3*M_PI/4);
				// ROS_WARN_STREAM("Changed added");
			}


			else if (msg->ranges.at(front) < PROXIMITY_RANGE_M) {
				fsm = FSM_ROTATE_2;
				ROS_INFO_STREAM("FSM = Rotate 2");
				ROS_INFO_STREAM("Counter-clockwise");

			}	
		}
	};

	// Main FSM loop for ensuring that ROS messgaes are processed in a timely
	// manner, and also for sending velocity contorls to the robot based on 
	// the FSM state
void spin() {
		ros::Rate rate(10); // Specify the FSM loop rate in Hz

		tf::TransformListener listener;
		tf::StampedTransform transform;


		while (ros::ok()){

			try {
				listener.lookupTransform("/base_link", "/odom", ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}
			// If switched goes here
			// if ( (g_switched != 0) and (g_switched % 2 == 0) ){
				// g_change = g_change +  (2*M_PI - 3*M_PI/4);
				// ROS_WARN_STREAM("Change added");
			// }

			double theta_act = tf::getYaw(transform.getRotation());
			double theta_des = 0 + g_change + ((g_switched % 2)*M_PI) ;
			double v, w;

			// TO DO
			if (fsm == FSM_HEADING_1) {
				v= FORWARD_SPEED_MPS;
				w= 2 * (angles::shortest_angular_distance(theta_des, theta_act));
				move(v, w);
			}
			if (fsm == FSM_HEADING_2) {
				v= FORWARD_SPEED_MPS;
				w= 2 * (angles::shortest_angular_distance(theta_des + M_PI, theta_act));
				move(v, w);
			}
			if (fsm == FSM_ROTATE_1) {
				v= FORWARD_SPEED_MPS/3;
				w= -v/RADIUS;
				move(v, w);
				if (angles::shortest_angular_distance((theta_des + M_PI), theta_act) >= 0.0 and fabs(angles::shortest_angular_distance((theta_des + M_PI), theta_act) < M_PI/9)){

					ROS_INFO_STREAM(angles::shortest_angular_distance((theta_des + M_PI), theta_act));
					fsm = FSM_HEADING_2;
					ROS_INFO_STREAM("FSM = Heading 2");
					ROS_INFO_STREAM("Robot is going towards " << 360.0 - fmod(((theta_des + M_PI)*180/M_PI),360.0) );
					ROS_INFO_STREAM("Change = " << (g_change*180/M_PI));


				}
			}
			if (fsm == FSM_ROTATE_2) {
				v= FORWARD_SPEED_MPS/3;
				w= v/RADIUS;
				move(v, w);
				if (angles::shortest_angular_distance(theta_des, theta_act) >= 0.0 and fabs(angles::shortest_angular_distance(theta_des, theta_act) < M_PI/9)){

					ROS_INFO_STREAM(angles::shortest_angular_distance(theta_des, theta_act));
					fsm = FSM_HEADING_1;
					ROS_INFO_STREAM("FSM = Heading 1");
					ROS_INFO_STREAM("Robot is going towards " << 360.0 - fmod((theta_des*180/M_PI),360.0) );
					ROS_INFO_STREAM("Change = " << (g_change*180/M_PI) );


				}
			}
			ros::spinOnce();
			rate.sleep();
		};
	};

	enum FSM{FSM_HEADING_1, FSM_HEADING_2, FSM_ROTATE_1, FSM_ROTATE_2};

// Tunable parameters 
// TO DO: tune parameters as you see fit
const static double MIN_SCAN_ANGLE_RAD= -20.0/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD= +20.0/180*M_PI;
const static float PROXIMITY_RANGE_M= 1; // Should be smaller than range_max
const static double FORWARD_SPEED_MPS= 1;
const static double ROTATE_SPEED_RADPS= M_PI/2;
const static double RADIUS= 0.3175; // Calculated from the length of the robot



protected:
	ros::Publisher commandPub;
	ros::Subscriber laserSub;
	enum FSM fsm;
	ros::Time rotateStartTime;
	ros::Duration rotateDuration;
};



int main(int argc, char **argv) {
	ros::init(argc, argv, "back_and_forth");
	ros::NodeHandle n;

	RandomWalk walker(n);
	walker.spin();

	return 0;
};

