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

int state;
char heardingBarNumber;

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
    if(heardingBarNumber == 18) {
	    if(px > 16 && state == 0) {
		    state = 1;
	    } else if(px > 4.7 && state == 2) {
		    state = 1;
	    }
    } else if(heardingBarNumber == 19) {
	     if(px < -18 && state == 0) {
		      state = 1;
	     } else if(px > 4.7 && state == 2) {
		      state = 1;
	      }	
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

    std::stringstream rName;
    rName.str("");
    rName << "RobotNode" << argv[1];

    ros::init(argc, argv, rName.str());
    ros::NodeHandle n;

    rName.str("");
    rName << "robot_" << argv[1]<<"/cmd_vel";
    ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>(rName.str(),1000);

    rName.str("");
    rName << "robot_" << argv[1]<<"/odom";
    ros::Subscriber StageOdom_sub = n.subscribe<nav_msgs::Odometry>(rName.str(),1000,StageOdom_callback);

    rName.str("");
    rName << "robot_" << argv[1]<<"/base_scan";
    ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>(rName.str(),1000,StageLaser_callback);

    ros::Rate loop_rate(10);

    //count of how many messages we have sent
    int count = 0;

    //Argument for opening or closing (second argument)
    state = atoi(argv[2]);
    //Argument for gate number (first argument)
    heardingBarNumber = atoi(argv[1]);

	  //velocity of this RobotNode
   	geometry_msgs::Twist RobotNode_cmdvel;

	  vector <instruction_struct> instruction_vector;

    //Conditions to check gate to move, and call addInstruction
    if(heardingBarNumber == 18) {
      addInstruction(instruction_vector, 75, 1, 0.0, 1);
  	  addInstruction(instruction_vector, 0, 0, 0.0, 2);
  	  addInstruction(instruction_vector, 75, 1, 0.0, 0);
    }else if(heardingBarNumber == 19) {
      addInstruction(instruction_vector, 75, -1, 0.0, 1);
  	  addInstruction(instruction_vector, 0, 0, 0.0, 2);
  	  addInstruction(instruction_vector, 75, 1, 0.0, 0);
    }


    //keep track of what step we are up to
    int current_step = 0;
    int current_step_count = 0;

    while (ros::ok())
    {
        //messages to stage
        linear_x = instruction_vector[current_step].linear_x;
        angular_z = instruction_vector[current_step].angular_z;
        ++current_step_count;

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










