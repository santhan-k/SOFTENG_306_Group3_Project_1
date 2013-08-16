#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"

#include <sstream>
#include "math.h"

using namespace std;

// Global variables and objects
ros::Publisher RobotNode_stage_pub;
geometry_msgs::Twist RobotNode_cmdvel;

bool showDebug = false;
vector<float> prevRanges;
#define SAMPLE_NUMBER 10 // represents number of samples in world file.

void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index);

void StageOdom_callback(nav_msgs::Odometry msg)
{

}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
  // TODO rotate to avoid object until a majority of the beams are free,
  // implying that path way forward is open for traversal.
  // TODO find a way to detect type of object that is in view or has collided with.

  if(showDebug)
    ROS_INFO("------------------------ Intensity ----------------------------");

  for(unsigned int i = 0; i < msg.intensities.size(); ++i){
    // Either 1 or 0 is returned. 1 if an object is view 0 otherwise.
    double curRange = msg.intensities[i];
    if(showDebug)
      ROS_INFO("Current intensity is    : %f", curRange);
  }

  if(showDebug)
    ROS_INFO("-------------------------- Range -----------------------------");

  int current_lowest_index = 0;
  double smallest_range = msg.ranges[current_lowest_index];
  prevRanges.push_back(msg.ranges[0]);
  // Iterate through LaserScan messages and find the smallest range
  for(unsigned int i = 1; i < msg.ranges.size(); ++i)
  {
    if(showDebug)
      ROS_INFO("Current range is    : %f", msg.ranges[i]);

    if(msg.ranges[current_lowest_index] > msg.ranges[i])
    {
      current_lowest_index = i;
      smallest_range = msg.ranges[current_lowest_index];
    }
    prevRanges.push_back(msg.ranges[i]);
  }  
  
  collisionAvoidance(smallest_range, msg, current_lowest_index);

  // Publish the message
  RobotNode_stage_pub.publish(RobotNode_cmdvel);

  if(showDebug)
    ROS_INFO("-------------------------- END -----------------------------");
}

// This function prevents robot(sheep) from colliding with stage objects.
void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index){
    
  if(showDebug)
    ROS_INFO("Lowest index: %d", current_lowest_index);

  // If the lowest range is less than 1.5 in length, the robot will begin
  // rotating to attempt to avoid the obstacle
  if(smallest_range < 1.5 && smallest_range > 0.8) //between 0.8 and 1.5 exclusive
  {
    // We are getting close to an obstacle, start turning
    if(showDebug)
      ROS_INFO("Collision at beam: %d | Range: %f", current_lowest_index, smallest_range);

    // Slow the robot down to 0.1m/s
    RobotNode_cmdvel.linear.x = 0.1;

    // Decide whether to turn anti-clockwise or clockwise depending on which
    // side of the robot is closest to the object
    if(current_lowest_index < SAMPLE_NUMBER/2) // sample_number is the number of beams
    {
      RobotNode_cmdvel.angular.z = (M_PI / 18) * 5; //anti-clockwise
    }
    else
    {
      RobotNode_cmdvel.angular.z = -(M_PI / 18) * 5; //clockwise
    }
  }
  else if(smallest_range <= 0.8) // less than 0.8 inclusive
  {
    // We are really close to an obstacle, stop moving forward and rotate on the spot
    if(showDebug)
      ROS_INFO("Collision at beam: %d | Range: %f", current_lowest_index, smallest_range);

    RobotNode_cmdvel.linear.x = 0;
    RobotNode_cmdvel.angular.z = (M_PI / 18) * 5; // just turn, don't care what direction
  }
  else
  {
    // If no potential collisions are detected, move forward normally
    RobotNode_cmdvel.linear.x = 1;
    RobotNode_cmdvel.angular.z = 0;
  }

}

int main(int argc, char** argv){
  std::stringstream rName;
  rName.str("");
  rName << "RobotNode" << argv[1];
  // You must call ros::init() first of all. ros::init() function needs to see argc and argv
  // The third argument is the name of the node
  ros::init(argc, argv, rName.str());
  ros::NodeHandle n;

  rName.str("");
  rName << "robot_" << argv[1]<<"/cmd_vel";
  RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>(rName.str(),1000);

  rName.str("");
  rName << "robot_" << argv[1]<<"/base_scan";
  ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>(rName.str(),1000,StageLaser_callback);

  rName.str("");
  rName << "robot_" << argv[1]<<"/odom";
  ros::Subscriber StageOdom_sub = n.subscribe<nav_msgs::Odometry>(rName.str(),1000,StageOdom_callback);

  int count = 0;
  ros::Rate r(10); // 10 cycles per second

  while(n.ok()){
    ++count;
    ros::spinOnce();
    r.sleep();
  }
}

