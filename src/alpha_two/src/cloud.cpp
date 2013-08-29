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


int velX = 0; //Initialise velocity x direction
int velY = 0; //Initialise velocity x direction
bool raining;


void StageOdom_cloudcallback(nav_msgs::Odometry msg)
{
  px = 30 + msg.pose.pose.position.x;
  py = 36 + msg.pose.pose.position.y;
  //printf("HELLO\n");
}

void StageRain_callback(alpha_two::rainFall msg)
{
  printf("RAINFALL = %d \n", msg.rain);
  if(msg.rain == 1)
  {
    raining = true;
  }
  else
  {
    raining = false;
  }
}


int main(int argc, char **argv)
{

  dayCounter = 0;
  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument   is the name of the node
  ros::init(argc, argv, "Cloud");

  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;
  
  
  ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_11/cmd_vel",1000);
  ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_11/odom",1000, StageOdom_cloudcallback); 
  ros::Rate loop_rate(10);
  

  ros::Subscriber rainFall_sub = n.subscribe<alpha_two::rainFall>("rain_msg", 1000, StageRain_callback); 

  //messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;
  bool reachedLimit = false;
  while (ros::ok())
  {
    if((px < -35 && py < -35) && !reachedLimit) //check the cloud Reached the lower limit
    {
      reachedLimit = true;
      velX = 1;
      velY = 1;
    }
    
    else if (!reachedLimit) //
    {
      velX = -1;
      velY = -1;
    }
    else if (px>35 && py>35) //check the cloud Reached the upper limit
    {
      reachedLimit = false;
    }
    if(raining)
    {
      RobotNode_cmdvel.linear.x = velX;
      RobotNode_cmdvel.linear.y = velY;
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
