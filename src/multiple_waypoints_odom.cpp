#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>


nav_msgs::Odometry g_odom;

void odom_CB(const nav_msgs::Odometry& msg){
  //Write published Odometry message to a global variable
  g_odom = msg;
}

void addWPtoRectangle(geometry_msgs::PoseArray original, geometry_msgs::PoseArray &added){
  ROS_INFO_STREAM("Function has been called");
  const double robWidth = 0.635;
  int numWP = original.poses.size();
  double height = (original.poses.at(1).position.y)-(original.poses.at(0).position.y);
  ROS_INFO_STREAM("");
  double width = (original.poses.at(2).position.x)-(original.poses.at(1).position.x);
  ROS_INFO_STREAM("Height = " << height << "      Width = " << width);
  int numSets = std::ceil(std::min(((height/robWidth)/2.0),((width/robWidth)/2.0)));

  ROS_INFO_STREAM("The number of WP is " << numWP);
  ROS_INFO_STREAM("The number of sets is " << numSets);

  for (int j=0; j<numSets; j++){
    for( int i=0; i<(numWP+1); i++){
        added.poses.push_back(original.poses.at(i%4));
        ROS_INFO_STREAM("Element added to added");
        } 
    }



    for (int i=numWP; i < added.poses.size(); i++){

      if( i%5 == 4){
        added.poses.at(i).position.x = added.poses.at(i-numWP).position.x;
        added.poses.at(i).position.y = added.poses.at(i-numWP).position.y;
      }

      else if ( i%5 == 0 ){
        added.poses.at(i).position.x = added.poses.at(i-(numWP+1)).position.x + (robWidth);
        added.poses.at(i).position.y = added.poses.at(i-(numWP+1)).position.y + (robWidth); 
      }

      else if ( i%5 == 1 ){
        added.poses.at(i).position.x = added.poses.at(i-(numWP+1)).position.x + (robWidth);
        added.poses.at(i).position.y = added.poses.at(i-(numWP+1)).position.y - (robWidth); 
      }

      else if ( i%5 == 2 ){
        added.poses.at(i).position.x = added.poses.at(i-(numWP+1)).position.x - (robWidth);
        added.poses.at(i).position.y = added.poses.at(i-(numWP+1)).position.y - (robWidth);
      }

      else {
        added.poses.at(i).position.x = added.poses.at(i-(numWP+1)).position.x - (robWidth);
        added.poses.at(i).position.y = added.poses.at(i-(numWP+1)).position.y + (robWidth);
      }
    }
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
  waypoint0.position.y = 0;  // y position of waypoint in meters
  arr.poses.push_back(waypoint0);

  geometry_msgs::Pose waypoint1;
  waypoint1.position.x = 0; // x position of waypoint in meters
  waypoint1.position.y = 10;  // y position of waypoint in meters
  arr.poses.push_back(waypoint1);

  geometry_msgs::Pose waypoint2;
  waypoint2.position.x = 2; // x position of waypoint in meters
  waypoint2.position.y = 10;  // y position of waypoint in meters
  arr.poses.push_back(waypoint2);

  geometry_msgs::Pose waypoint3;
  waypoint3.position.x = 2; // x position of waypoint in meters
  waypoint3.position.y = 0;  // y position of waypoint in meters
  arr.poses.push_back(waypoint3);

  geometry_msgs::PoseArray newOne;

  addWPtoRectangle(arr, newOne);



  ROS_INFO_STREAM("The size of this pose array is " << newOne.poses.size());

  for ( int i=0; i < newOne.poses.size(); i++){
    ROS_INFO_STREAM("This is is waypoint number " << i);
    ROS_INFO_STREAM("("<< newOne.poses.at(i).position.x <<"," << newOne.poses.at(i).position.y <<")");
    ROS_INFO_STREAM("                                                      ");
  }

  //This is the advertising  object that will publish the desired forward and angular velocity of the robot.
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;

  // This is a subscriber object that will subscribe to the robot's odom topic
  ros::Subscriber odom_sub = nh.subscribe("odom",1,odom_CB);

  // This creates a transform listener object 

  // Set the rate of the while loop
  ros::Rate rate(10.0);
  while (nh.ok()){
    for(int i=0; i<newOne.poses.size(); i++){
      double xWP = newOne.poses.at(i).position.x;
      double yWP = newOne.poses.at(i).position.y;
      double xR = g_odom.pose.pose.position.x;
      double yR = g_odom.pose.pose.position.y;
      double conv = 180.0/3.14159;

      while(fabs(sqrt((xWP-xR)*(xWP-xR)+(yWP-yR)*(yWP-yR)))>0.1 and nh.ok()){
        ros::spinOnce();
        xR = g_odom.pose.pose.position.x;
        yR = g_odom.pose.pose.position.y;

        ROS_INFO_STREAM("This is waypoint number " << i);

        ROS_INFO_STREAM("The robot position is (" << xR << "," << yR << ").");

        ROS_INFO_STREAM("The waypoint position is (" << xWP << "," << yWP << ").");

        // Determine the desired yaw angle
        double theta_des = atan2(yWP-yR,xWP-xR);
        // Determin the actual yaw angle
        double theta_act = tf::getYaw(g_odom.pose.pose.orientation);

        ROS_INFO_STREAM("Actual = " << theta_act*conv << "   Desired = " << theta_des*conv);
        ROS_INFO_STREAM("                                                     ");
          //Simple algorithm to steer robot to a point (not orientation).
        vel.angular.z = 4*angles::shortest_angular_distance(theta_act, theta_des);
        //vel.linear.x = 1.0*sqrt((xWP-xR)*(xWP-xR)+(yWP-yR)*(yWP-yR));

        ROS_INFO_STREAM("The angular distance is " << fabs(angles::shortest_angular_distance(theta_act, theta_des))*conv);

        if (fabs(angles::shortest_angular_distance(theta_act, theta_des))>(2.0/conv)){
          vel.linear.x = 0;
          //vel.angular.z = 1*((theta_des - theta_act)*conv);
        }
        else{ 
          vel.linear.x = 1.0*sqrt((xWP-xR)*(xWP-xR)+(yWP-yR)*(yWP-yR));
          if ( vel.linear.x > 2.0){
            vel.linear.x = 2;
          }
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
