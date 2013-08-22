#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"

#include <sstream>
#include "math.h"

// ===================
// West Gate - robot_6
// ===================

using namespace std;

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double ptheta;
double theta;

int state;


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
	
    //condition for movement
    if(px > 11 && state == 0) {
        state = 1;
    } else if(px < 5 && state == 2) {
        state = 1;
    }	

    //displayed on terminal
    ROS_INFO("Current x position is: %f", px);
    ROS_INFO("Current y position is: %f", py);
}



void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number	
}

void addInstruction(vector<instruction_struct>& instruction_vector, int step_count, double linear_x, double angular_z, int next_step)
{
  instruction_struct *Instruction = new instruction_struct; //create a new instruction_struct
  Instruction->step_count = step_count;
  Instruction->linear_x = linear_x;
  Instruction->angular_z = angular_z;
  Instruction->next_step = next_step;
  instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector
}

int main(int argc, char **argv)
{

    //initialize robot parameters
    //Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
    theta = M_PI/2.0;
    px = 26;
    py = 0;
	
    //Initial velocity
    linear_x = 0;
    angular_z = 0;
	
    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
    ros::init(argc, argv, "RobotNode0");

    //NodeHandle is the main access point to communicate with ros.
    ros::NodeHandle n;

    //advertise() function will tell ROS that you want to publish on a given topic_
    //to stage
    ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_9/cmd_vel",1000); 

    //subscribe to listen to messages coming from stage
    ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_9/odom",1000, StageOdom_callback);
    ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_9/base_scan",1000,StageLaser_callback);

    ros::Rate loop_rate(10);


    //a count of how many messages we have sent
    int count = 0;

    state = atoi(argv[1]);

    ////messages
    //velocity of this RobotNode
   	 geometry_msgs::Twist RobotNode_cmdvel;

	vector <instruction_struct> instruction_vector;
  	addInstruction(instruction_vector, 75, 1, 0.0, 1);
  	addInstruction(instruction_vector, 0, 0, 0.0, 2);
 	addInstruction(instruction_vector, 75, -1, 0.0, 0);


    //keep track of what step we are up to
    int current_step = 0;
    int current_step_count = 0;


    while (ros::ok())
    {
        //messages to stage
        linear_x = instruction_vector[current_step].linear_x;
        angular_z = instruction_vector[current_step].angular_z;
        ++current_step_count;
	//	if(current_step_count == instruction_vector[current_step].step_count) {
	//		current_step = instruction_vector[current_step].next_step;
	//		current_step_count = 0;
	//	}

        if(state == 0) {
            linear_x = instruction_vector[0].linear_x;
            angular_z = instruction_vector[0].angular_z;
            } else if(state == 1){
                break;
            } else if(state == 2) {
                linear_x = instruction_vector[2].linear_x;
                angular_z = instruction_vector[2].angular_z;
        }

        RobotNode_cmdvel.linear.x = linear_x;
        RobotNode_cmdvel.angular.z = angular_z;
        
        cout << "Current step: " << current_step << "\n";

        //publish the message
        RobotNode_stage_pub.publish(RobotNode_cmdvel);
	
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    } //ends while()
	
    RobotNode_cmdvel.linear.x = 0;
    RobotNode_cmdvel.angular.z =0;
        
    cout << "Current step: " << current_step << "\n";

	//publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
    return 0;

} //ends main()

