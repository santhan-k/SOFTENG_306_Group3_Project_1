#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/grassState.h"
#include "alpha_two/sheepState.h"
#include "alpha_two/farmState.h"
#include <sstream>
#include "math.h"

using namespace std;

alpha_two::farmState new_farm_msg;

int main(int argc, char **argv)
{
	
  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument   is the name of the node
  ros::init(argc, argv, "Farm_Control");

  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;

  //advertise() function will tell ROS that you want to publish on a given topic_
  //to stage

  ros::Publisher farmNode_pub = n.advertise<alpha_two::farmState>("farm_msg", 1000);

  ros::Rate loop_rate(10);

  ////messages
  //velocity of this RobotNode
  //geometry_msgs::Twist RobotNode_cmdvel;
  while (ros::ok())
  {
	  //newmsg.x = initialPosx;
	  //newmsg.y = initialPosy;

	  //publish the message
	  //grassNode_pub.publish(newmsg);
	  
	  
	  
	  ros::spinOnce();

	  loop_rate.sleep();
  }

return 0;

}

