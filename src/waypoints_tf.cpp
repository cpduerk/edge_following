#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>


int main(int argc, char** argv){
  //Initialize the node
  ros::init(argc, argv, "waypoints_tf");
  ros::NodeHandle nh;

  //This is the advertising  object that will publish the desired forward and angular velocity of the robot.
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  // Initialize the velocity command that we will publish.
  geometry_msgs::Twist vel;

  // This creates a transform listener object 
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // Set the rate of the while loop
  ros::Rate rate(10.0);

  std::vector<std::string> wpList;
  wpList.push_back("/wp0");
  wpList.push_back("/wp1");
  wpList.push_back("/wp2");
  wpList.push_back("/wp3");

  double d;



  while (nh.ok()){

    for(int i=0; i<wpList.size(); i++){

      d=1;

      while(d > 0.1 and nh.ok()){

        try {
          listener.lookupTransform("/base_link", wpList.at(i), ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
        }

        //Simple algorithm to steer robot to a point (not orientation).
        vel.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
        d = sqrt(pow(transform.getOrigin().x(),2) + pow(transform.getOrigin().y(),2));
        vel.linear.x = 0.5 * d;

          //Publish velocity command.
          vel_pub.publish(vel);
          //And sleep for the rest of the loop cycle.
          rate.sleep();
      }
    }
  }
  return 0;
}
