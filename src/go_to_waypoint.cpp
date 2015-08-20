#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
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


  // Set a hardcoded wayoint for the robot to travel to
  geometry_msgs::Pose waypoint;
  waypoint.position.x = 20; // x position of waypoint in meters
  waypoint.position.y = -10;  // y position of waypoint in meters

  //This is the advertising  object that will publish the desired forward and angular velocity of the robot.
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;

  // This is a subscriber object that will subscribe to the robot's odom topic
  ros::Subscriber odom_sub = nh.subscribe("odom",1,odom_CB);

  // Set the rate of the while loop
  ros::Rate rate(10.0);

  while (nh.ok()){
    ros::spinOnce();
    double xWP = waypoint.position.x;
    double yWP = waypoint.position.y;
    double xR = g_odom.pose.pose.position.x;
    double yR = g_odom.pose.pose.position.y;

    ROS_INFO_STREAM("The robot position is (" << xR << "," << yR << ").");

    ROS_INFO_STREAM("The waypoint position is (" << xWP << "," << yWP << ").");

    // Determine the desired yaw angle
    double theta_des = atan2(yWP-yR,xWP-xR);
    // Determin the actual yaw angle
    double theta_act = tf::getYaw(g_odom.pose.pose.orientation);

      //Simple algorithm to steer robot to a point (not orientation).
    vel.angular.z = 1.0*(theta_des - theta_act);
    vel.linear.x = 1.0*sqrt((xWP-xR)*(xWP-xR)+(yWP-yR)*(yWP-yR));

      //Publish velocity command.
      vel_pub.publish(vel);
      //And sleep for the rest of the loop cycle.
      rate.sleep();
    }
  return 0;
}
