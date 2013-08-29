#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/grassState.h"
#include "alpha_two/sheepState.h"
#include <sstream>
#include "math.h"

using namespace std;

//velocity of the robot
double linear_x;
double linear_y;
double angular_z;

//pose of the robot
double px;
double py;
double ptheta;
double theta;

int fieldNumber;

struct instruction_struct{
  int step_count; //how many times do we execute this step before we move to the next step
  double linear_x; //linear velocity in m/s
  double linear_y; //angular velocity
  int next_step; //which step do we move to when we finish this current step
};

void StageOdom_callback(nav_msgs::Odometry msg){
  //This is the call back function to process odometry messages coming from Stage.
  px = 5 + msg.pose.pose.position.x;
  py =10 + msg.pose.pose.position.y;
  //ptheta = fmod((2*M_PI) + theta + angles::normalize_angle_positive(asin(msg.pose.pose.orientation.z) * 2), 2*M_PI) * (180/M_PI);
  ROS_INFO("Current x position is: %f", px);
  ROS_INFO("Current y position is: %f", py);
  ROS_INFO("Current theta is: %f", ptheta);
}

void StageGrass_callback(alpha_two::grassState msg){
 //ROS_INFO("RECEIVED GRASS MESSAGE FROM: %d",msg.G_ID);
	
}

//void StageLaser_callback(sensor_msgs::LaserScan msg)
//{
//  //This is the callback function to process laser scan messages
//  //you can access the range data from msg.ranges[i]. i = sample number
//
//}

void addInstruction(vector<instruction_struct>& instruction_vector, int step_count, double linear_x, double linear_y, int next_step){
  instruction_struct *Instruction = new instruction_struct; //create a new instruction_struct
  Instruction->step_count = step_count;
  Instruction->linear_x = linear_x;
  Instruction->linear_y = linear_y;
  Instruction->next_step = next_step;
  instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector
}

int main(int argc, char **argv){
  //initialize robot parameters
  //Initial pose. This is same as the pose that you used in the world file to set the robot pose.
  theta = M_PI/2.0;
  px = 3;
  py = 27.5;

  //Initial velocity
  linear_x = 0;
  linear_y = 0;

  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
  ros::init(argc, argv, "RobotNode0");

  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;

  //advertise() function will tell ROS that you want to publish on a given topic_
  //to stage
  ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);

  //subscribe to listen to messages coming from stage
  ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
  //ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

  ros::Subscriber grassNode_sub = n.subscribe<alpha_two::grassState>("Grass_msg", 1000, StageGrass_callback); 

  ros::Rate loop_rate(10);

  //a count of howmany messages we have sent
  int count = 0;

  fieldNumber = atoi(argv[1]);

  ////messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;

  // Sample instructions
  vector <instruction_struct> instruction_vector; //create new vector of type instruction_struct

  if(fieldNumber == 1) {
      addInstruction(instruction_vector, 2600, 0.10, -0.10, 1); // Diagonal movement
      addInstruction(instruction_vector, 78, 0, -0.5, 0);     // Move down
  } else if(fieldNumber == 2) {
      addInstruction(instruction_vector, 2600, -0.10, -0.10, 1); // Diagonal movement
      addInstruction(instruction_vector, 75, -0.5, 0, 0);     // Move down
  } else if(fieldNumber == 3) {
      addInstruction(instruction_vector, 2600, -0.10, 0.10, 1); // Diagonal movement
      addInstruction(instruction_vector, 75, 0, 0.5, 0);     // Move down
  } else if(fieldNumber == 4) {
      addInstruction(instruction_vector, 2600, 0, 0.10, 1); // Diagonal movement
      addInstruction(instruction_vector, 75, 0, 0, 0);     // Move down
  } 
      
  //addInstruction(instruction_vector, 75, 0, 0, 0);
//  addInstruction(instruction_vector, 0, 0, 0.0, 3); // Stop movement
//  addInstruction(instruction_vector, 75, 1, 0.0, 0); // Outward movement

  //keep track of what step we are up to
  int current_step = 0;
  int current_step_count = 0;

  while (ros::ok()){
    //messages to stage
    linear_x = instruction_vector[current_step].linear_x;
    linear_y = instruction_vector[current_step].linear_y;
    ++current_step_count;


    if(instruction_vector[current_step].next_step == 0 && current_step_count == instruction_vector[current_step].step_count) {
       break;
    }

    if(current_step_count == instruction_vector[current_step].step_count){
      current_step = instruction_vector[current_step].next_step;
      current_step_count = 0;
    }


    RobotNode_cmdvel.linear.x = linear_x;
    RobotNode_cmdvel.linear.y = linear_y;

    cout << "Current step: " << current_step << "\n";

    //publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  RobotNode_cmdvel.linear.x = 0;
  RobotNode_cmdvel.linear.y =0;

  RobotNode_stage_pub.publish(RobotNode_cmdvel);

  return 0;
}

