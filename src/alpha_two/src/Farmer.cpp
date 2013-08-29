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

struct instruction_struct
{
  int step_count; //how many times do we execute this step before we move to the next step
  double linear_x; //linear velocity in m/s
  double linear_y; //angular velocity
  int next_step; //which step do we move to when we finish this current step
};

void addInstruction(vector<instruction_struct>& instruction_vector, int step_count, double linear_x, double linear_y, int next_step)
{
  instruction_struct *Instruction = new instruction_struct; //create a new instruction_struct
  Instruction->step_count = step_count;
  Instruction->linear_x = linear_x;
  Instruction->linear_y = linear_y;
  Instruction->next_step = next_step;
  instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector
}

int main(int argc, char **argv)
{
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

  // Subscribes to grass messages
  ros::Subscriber grassNode_sub = n.subscribe<alpha_two::grassState>("Grass_msg", 1000, StageGrass_callback); 

  ros::Rate loop_rate(10);

  //a count of how many messages we have sent
  int count = 0;

  fieldNumber = atoi(argv[1]);

  ////messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;

  // Sample instructions
  vector <instruction_struct> instruction_vector; //create new vector of type instruction_struct

  if(fieldNumber == 1) 
  {
      addInstruction(instruction_vector, 2600, 0.10, -0.10, 1); // Diagonal movement
      addInstruction(instruction_vector, 78, 0, -0.5, 0);       // Move down
  } 
  else if(fieldNumber == 2) 
  {
      addInstruction(instruction_vector, 2600, -0.10, -0.10, 1); // Diagonal movement
      addInstruction(instruction_vector, 75, -0.5, 0, 0);        // Move to the left
  }
  else if(fieldNumber == 3) 
  {
      addInstruction(instruction_vector, 2600, -0.10, 0.10, 1); // Diagonal movement
      addInstruction(instruction_vector, 75, 0, 0.5, 0);        // Move up
  }
  else if(fieldNumber == 4) 
  {
      addInstruction(instruction_vector, 2600, 0, 0.10, 1);     // Move up
      addInstruction(instruction_vector, 75, 0, 0, 0);          // Stops
  } 
      
  //keep track of what step we are up to
  int current_step = 0;
  int current_step_count = 0;

  while (ros::ok())
  {
    //messages to stage
    linear_x = instruction_vector[current_step].linear_x;
    linear_y = instruction_vector[current_step].linear_y;
    ++current_step_count;

    // Checks to see if the last instruction in the instruction vector has been executed.
    if(instruction_vector[current_step].next_step == 0 && current_step_count == instruction_vector[current_step].step_count) 
    {
       break;   // Stop farmer
    }

    // Goes into next step when the current step finishes.
    if(current_step_count == instruction_vector[current_step].step_count)
    {
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

  RobotNode_cmdvel.linear.x = 0;    // Change velecity back to zero
  RobotNode_cmdvel.linear.y =0;     // Change velecity back to zero

  RobotNode_stage_pub.publish(RobotNode_cmdvel);    //publish changed velocities

  return 0;
}

