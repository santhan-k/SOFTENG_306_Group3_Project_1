#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"

#include <sstream>
#include "math.h"

using namespace std;

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double ptheta;
double theta;

struct instruction_struct
{
	int step_count; //how many times do we execute this step before we move to the next step
	double linear_x; //linear velocity in m/s
	double angular_z; //angular velocity
	int next_step; //which step do we move to when we finish this current step
};

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 5 + msg.pose.pose.position.x;
	py =10 + msg.pose.pose.position.y;
	ptheta = fmod((2*M_PI) + theta + angles::normalize_angle_positive(asin(msg.pose.pose.orientation.z) * 2), 2*M_PI) * (180/M_PI);
	ROS_INFO("Current x position is: %f", px);
	ROS_INFO("Current y position is: %f", py);
	ROS_INFO("Current theta is: %f", ptheta);
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

int main(int argc, char **argv)
{

 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = 10;
	py = 20;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "RobotNode0");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

// Step 0
vector <instruction_struct> instruction_vector; //create new vector of type instruction_struct
instruction_struct *Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 250; //add data
Instruction->linear_x = 4; //add data
Instruction->angular_z = 0.0; //add data
Instruction->next_step = 1; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

// Step 1
Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 18; //add data
Instruction->linear_x = 0; //add data
Instruction->angular_z = -(M_PI / 18) * 5; //(M_PI / 18) = 1degree, * 5 = 5 degrees
Instruction->next_step = 2; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

// Step 2
Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 50; //add data
Instruction->linear_x = 4; //add data
Instruction->angular_z = 0.0; //add data
Instruction->next_step = 3; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

// Step 3
Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 18; //add data
Instruction->linear_x = 0.0; //add data
Instruction->angular_z = -(M_PI / 18) * 5; //(M_PI / 18) = 1degree, * 5 = 5 degrees
Instruction->next_step = 4; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

// Step 4
Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 220; //add data
Instruction->linear_x = 4; //add data
Instruction->angular_z = 0.0; //add data
Instruction->next_step = 5; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

// Step 5
Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 18; //add data
Instruction->linear_x = 0.0; //add data
Instruction->angular_z = (M_PI / 18) * 5; //(M_PI / 18) = 1degree, * 5 = 5 degrees
Instruction->next_step = 6; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

// Step 6
Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 50; //add data
Instruction->linear_x = 4; //add data
Instruction->angular_z = 0.0; //add data
Instruction->next_step = 7; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

// Step 7
Instruction = new instruction_struct; //create a new instruction_struct
Instruction->step_count = 18; //add data
Instruction->linear_x = 0.0; //add data
Instruction->angular_z = (M_PI / 18) * 5; //(M_PI / 18) = 1degree, * 5 = 5 degrees
Instruction->next_step = 0; //add data
instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector

//keep track of what step we are up to
int current_step = 0;
int current_step_count = 0;

while (ros::ok())
{
	//messages to stage
	linear_x = instruction_vector[current_step].linear_x;
	angular_z = instruction_vector[current_step].angular_z;
	++current_step_count;
	if(current_step_count == instruction_vector[current_step].step_count) {
		current_step = instruction_vector[current_step].next_step;
		current_step_count = 0;
	}

	RobotNode_cmdvel.linear.x = linear_x;
	RobotNode_cmdvel.angular.z = angular_z;
        
	cout << "Current step: " << current_step << "\n";

	//publish the message
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}

return 0;

}

