#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/grassState.h"
#include "alpha_two/sheepState.h"
#include "alpha_two/sheepDogState.h"
#include <sstream>
#include "math.h"

using namespace std;
#define PI 3.14159265
// Global variables and objects
nav_msgs::Odometry sheepDog_odom;
ros::Publisher RobotNode_stage_pub;
ros::Publisher sheepDog_Pub;
geometry_msgs::Twist RobotNode_cmdvel; 

bool showDebug = false; //Show/hide ROS log messages
vector<float> previous_ranges;
#define SAMPLE_NUMBER 10 // represents number of samples in world file.
double px;
double py;
double ptheta;
double theta;
double grassX;
double grassY;
double initialPosx;
double initialPosy;
double initialTheta;
alpha_two::sheepDogState sheepDog_msg;

void SheepDogPosition_callback(nav_msgs::Odometry msg)
{
	py = msg.pose.pose.position.x;
  	px = msg.pose.pose.position.y;
	theta = msg.pose.pose.orientation.z;
	theta = floorf(theta * 1000) / 1000;  //Rounding to 3dp
	//ROS_INFO("INITIAL THETA = %f",initialTheta);
	//ROS_INFO("px: %f	py: %f		theta: %f", px, py, theta);
        
        sheepDog_msg.x = px;
        sheepDog_msg.y = py;
        sheepDog_Pub.publish(sheepDog_msg);
        ROS_INFO("X = %f",px);
        ROS_INFO("Y = %f",py);        
}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
   // Nothing to do here
}

int main(int argc, char** argv){
  ros::init(argc, argv, "SheepDog1");
  ros::NodeHandle n;
   
  //RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>(rName.str(),1000);
  sheepDog_Pub = n.advertise<alpha_two::sheepDogState>("sheepDog1_msg",1000);

  ros::Subscriber sheepDog_odom = n.subscribe<nav_msgs::Odometry>("/robot_13/base_pose_ground_truth",1000, SheepDogPosition_callback);

  
  //ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>(rName.str(),1000,StageLaser_callback);
  
  int count = 0;
  ros::Rate r(10); // 10 cycles per second

  while(n.ok()){
    ++count;    
    ros::spinOnce(); //Must Have this statement in the program
    r.sleep();
  }
}

