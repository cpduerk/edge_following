#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

nav_msgs::Odometry g_odom;

void odom_CB(const nav_msgs::Odometry& msg){
  //Write published Odometry message to a global variable
  g_odom = msg;
}

int main(int argc, char** argv){
  //Initialize the node
  ros::init(argc, argv, "go_to_waypoint");
  ros::NodeHandle nh;

  //Initialize the PoseArray
  geometry_msgs::PoseArray arr;
  // Set a hardcoded wayoint for the robot to travel to
  geometry_msgs::Pose waypoint0;
  waypoint0.position.x = 0; // x position of waypoint in meters
  waypoint0.position.y = 10;  // y position of waypoint in meters
  arr.poses.push_back(waypoint0);

  geometry_msgs::Pose waypoint1;
  waypoint1.position.x = 2; // x position of waypoint in meters
  waypoint1.position.y = 10;  // y position of waypoint in meters
  arr.poses.push_back(waypoint1);

  geometry_msgs::Pose waypoint2;
  waypoint2.position.x = 2; // x position of waypoint in meters
  waypoint2.position.y = 0;  // y position of waypoint in meters
  arr.poses.push_back(waypoint2);

  geometry_msgs::Pose waypoint3;
  waypoint3.position.x = 0; // x position of waypoint in meters
  waypoint3.position.y = 0;  // y position of waypoint in meters
  arr.poses.push_back(waypoint3);


  //This is the advertising  object that will publish the desired forward and angular velocity of the robot.
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;

  // This is a subscriber object that will subscribe to the robot's odom topic
  ros::Subscriber odom_sub = nh.subscribe("odom",1,odom_CB);

  // Set the rate of the while loop
  ros::Rate rate(10.0);
  while (nh.ok()){
    for(int i=0; i<arr.poses.size(); i++){
      double xWP = arr.poses.at(i).position.x;
      double yWP = arr.poses.at(i).position.y;
      double xR = g_odom.pose.pose.position.x;
      double yR = g_odom.pose.pose.position.y;
      double conv = 180.0/3.14159;

      while(sqrt((xWP-xR)*(xWP-xR)+(yWP-yR)*(yWP-yR))>0.1 and nh.ok()){
        ros::spinOnce();
        xR = g_odom.pose.pose.position.x;
        yR = g_odom.pose.pose.position.y;

        ROS_INFO_STREAM("The robot position is (" << xR << "," << yR << ").");

        ROS_INFO_STREAM("The waypoint position is (" << xWP << "," << yWP << ").");

        // Determine the desired yaw angle
        double theta_des = atan2(yWP-yR,xWP-xR);
        // Determin the actual yaw angle
        double theta_act = tf::getYaw(g_odom.pose.pose.orientation);

        ROS_INFO_STREAM("ACtual = " << theta_act*conv << "   Desired = " << theta_des*conv);
        ROS_INFO_STREAM("                                                     ");
          //Simple algorithm to steer robot to a point (not orientation).
        vel.angular.z = 1*((theta_des - theta_act)*conv);
        //vel.linear.x = 1.0*sqrt((xWP-xR)*(xWP-xR)+(yWP-yR)*(yWP-yR));

        if (abs((theta_act-theta_des))>2.0/conv){
          vel.linear.x = 0;
          //vel.angular.z = 1*((theta_des - theta_act)*conv);
        }
        else{ 
          vel.linear.x = 4.0*sqrt((xWP-xR)*(xWP-xR)+(yWP-yR)*(yWP-yR));
          // vel.angular.z = 1*((theta_des - theta_act)*conv); 
        }
          //Publish velocity command.
          vel_pub.publish(vel);
          //And sleep for the rest of the loop cycle.
          rate.sleep();
      }
    }
  }
  return 0;
}
