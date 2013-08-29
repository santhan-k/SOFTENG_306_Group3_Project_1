#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/grassState.h"
#include "alpha_two/sheepState.h"
#include "alpha_two/farmState.h"
#include "alpha_two/rainFall.h"
#include <sstream>
#include "math.h"

using namespace std;

alpha_two::farmState new_farm_msg;

int dayCounter;
double px,py;


int velX = 0;
int velY = 0;
bool raining;


void StageOdom_cloudcallback(nav_msgs::Odometry msg)
{
  px = msg.pose.pose.position.x;
  py = msg.pose.pose.position.y;
  //printf("HELLO\n");
}

int main(int argc, char **argv)
{

  dayCounter = 0;
  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument   is the name of the node
  ros::init(argc, argv, "Cloud");

  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;
  
  
  ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_10/cmd_vel",1000);
  ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_10/odom",1000, StageOdom_cloudcallback); 
  ros::Rate loop_rate(10);


  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;
  bool reachedLimit = false;
  while (ros::ok())
  {
    if(px<40)
    {
      RobotNode_cmdvel.linear.x = -1;
      RobotNode_cmdvel.linear.y = 0;
    }
    else
    {
      RobotNode_cmdvel.linear.x = 0;
      RobotNode_cmdvel.linear.y = 0;
    }

    //publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
     
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
