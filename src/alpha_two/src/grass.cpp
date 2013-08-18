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
double angular_z;

//pose of the robot
double px;
double py;
double ptheta;
double theta;
alpha_two::grassState newmsg;
int state;
double initialPosx;
double initialPosy;
double initialTheta;
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
	//float onex = msg.pose.pose.orientation.x;
	//float oney = msg.pose.pose.orientation.y;
	//printf("px = %f   py = %f x = %f  y = %f",px, py, onex, oney);
}

void StageSheep_callback(alpha_two::sheepState msg)
{
	 //ROS_INFO("RECEIVED Sheep MESSAGE FROM: %d",msg.S_ID);
	 if(msg.S_State==1 && msg.grass_locked==newmsg.G_ID && newmsg.G_State == 0){
		newmsg.G_State=1;
		newmsg.lockedBy = msg.S_ID;
		ROS_INFO("LOCKED by SHEEP: %d",msg.S_ID);
	}

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
	px = 26;
	py = 0;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
std::stringstream rName;
rName.str("");
rName << "GrassNode" << argv[1];
ros::init(argc, argv, rName.str());


//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage

rName.str("");
rName << "robot_" << argv[1]<<"/cmd_vel";
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>(rName.str(),1000); 

ros::Publisher grassNode_pub = n.advertise<alpha_two::grassState>("Grass_msg", 1000);

ros::Subscriber sheepState_sub = n.subscribe<alpha_two::sheepState>("Sheep_msg", 1000, StageSheep_callback); 

//subscribe to listen to messages coming from stage
rName.str("");
rName << "robot_" << argv[1]<<"/odom";
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>(rName.str(),1000, StageOdom_callback);

rName.str("");
rName << "robot_" << argv[1]<<"/base_scan";
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>(rName.str(),1000,StageLaser_callback);

ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

state = atoi(argv[1]);

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;
newmsg.G_State = 0;
newmsg.G_ID = atoi(argv[1]);
initialPosx = atoi(argv[2]);
initialPosy = atoi(argv[3]);
while (ros::ok())
{
	
	newmsg.x = initialPosx;
	newmsg.y = initialPosy;

	//publish the message
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
	grassNode_pub.publish(newmsg);
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}

return 0;

}

