#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <cstdlib>



void addWPtoRectangle(geometry_msgs::PoseArray original, geometry_msgs::PoseArrayPtr &added){
  const double robWidth = 0.635;
  int numWP = original.poses.size();
  double height = (original.poses.at(1).position.y)-(original.poses.at(0).position.y);
  double width = (original.poses.at(2).position.x)-(original.poses.at(1).position.x);
  int numSets = std::ceil(std::min(((height/robWidth)/2),((width/robWidth)/2)));

	for (int i=0; i<numSets; i++){
		for( int i=0; i<numWP; i++){
    		added->poses.push_back(original.poses.at(i));
    		} 
    }

    for (int i=numWP; i < added->poses.size(); i++){

    	if( i%5 == 4){
    		added->poses.at(i).position.x = added->poses.at(i-numWP).position.x;
    		added->poses.at(i).position.y = added->poses.at(i-numWP).position.y;
    	}

    	else if ( i%5 == 0 ){
    		added->poses.at(i).position.x = added->poses.at(i-numWP+1).position.x + (2*robWidth);
    		added->poses.at(i).position.y = added->poses.at(i-numWP+1).position.y + (2*robWidth);	
    	}

    	else if ( i%5 == 1 ){
    		added->poses.at(i).position.x = added->poses.at(i-numWP+1).position.x + (2*robWidth);
    		added->poses.at(i).position.y = added->poses.at(i-numWP+1).position.y - (2*robWidth);	
    	}

    	else if ( i%5 == 2 ){
    		added->poses.at(i).position.x = added->poses.at(i-numWP+1).position.x - (2*robWidth);
    		added->poses.at(i).position.y = added->poses.at(i-numWP+1).position.y - (2*robWidth);
    	}

    	else {
    		added->poses.at(i).position.x = added->poses.at(i-numWP+1).position.x - (2*robWidth);
    		added->poses.at(i).position.y = added->poses.at(i-numWP+1).position.y + (2*robWidth);
    	}
    }
}



int main() {
  
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

geometry_msgs::PoseArrayPtr newOne;

addWPtoRectangle(arr, newOne);




return 0;
}