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
int dayCounter;
double px,py;

// Changes the weather conditions of the farm
void changeWeather(){
  
  dayCounter = dayCounter%732;
  dayCounter++;
  //Rainfall is determined randomly according to the current season.
  //Spring
  if(dayCounter < 200){

    new_farm_msg.rainfall = rand()%3;

  //Winter
  }else if (dayCounter < 460){
    new_farm_msg.rainfall = rand()%4;

  //Summer
  }else if (dayCounter < 532){        
    new_farm_msg.rainfall = rand()%1; 
  //Autumn
  }else if (dayCounter < 732){        
    new_farm_msg.rainfall = rand()%2;
  }

  
  //Weather changes applied to each field.  
  new_farm_msg.f1_soil_condition = abs((new_farm_msg.f1_soil_condition +new_farm_msg.rainfall)%100);
  new_farm_msg.f2_soil_condition = abs((new_farm_msg.f2_soil_condition +new_farm_msg.rainfall)%100);
  new_farm_msg.f3_soil_condition = abs((new_farm_msg.f3_soil_condition +new_farm_msg.rainfall)%100);
  new_farm_msg.f4_soil_condition = abs((new_farm_msg.f4_soil_condition +new_farm_msg.rainfall)%100);
  
  
  //new_farm_msg.f4_soil_condition += int(float(new_farm_msg.f4_soil_condition)*(float(new_farm_msg.rainfall)/100.0)) -5.0;

}

void StageOdom_cloudcallback(nav_msgs::Odometry msg){
  px = 30 + msg.pose.pose.position.x;
  py = 36 + msg.pose.pose.position.y;
}


int main(int argc, char **argv)
{
	dayCounter = 0;
  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument   is the name of the node
  ros::init(argc, argv, "Farm_Control");

  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;

  //advertise() function will tell ROS that you want to publish on a given topic_
  //to stage

  ros::Publisher farmNode_pub = n.advertise<alpha_two::farmState>("farm_msg", 1000);
  
  ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_11/cmd_vel",1000);
  ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_11/odom",1000, StageOdom_cloudcallback); 
  ros::Rate loop_rate(10);
  
  new_farm_msg.rainfall = 0;
  new_farm_msg.f1_soil_condition = 100;
   
  new_farm_msg.f2_soil_condition = 50;
  
  new_farm_msg.f3_soil_condition = 70;
   
  new_farm_msg.f4_soil_condition = 85;
  ////messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;
  while (ros::ok())
  {
 
    //publish the message
	  //grassNode_pub.publish(newmsg);

    changeWeather();
    if(px < -60 && py < -60){
      RobotNode_cmdvel.linear.x = 0;
      RobotNode_cmdvel.linear.y = 0;
    }else {
      RobotNode_cmdvel.linear.x = -1;
      RobotNode_cmdvel.linear.y = -1;
    }

    RobotNode_stage_pub.publish(RobotNode_cmdvel);
	  farmNode_pub.publish(new_farm_msg);
	  
	  
	  ros::spinOnce();

	  loop_rate.sleep();
  }

return 0;

}

