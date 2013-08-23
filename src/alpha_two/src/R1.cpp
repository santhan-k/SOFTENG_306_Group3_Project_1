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
// Global variables and objects
ros::Publisher RobotNode_stage_pub;
geometry_msgs::Twist RobotNode_cmdvel;

bool showDebug = false; //Show/hide ROS log messages
#define SAMPLE_NUMBER 10 // represents number of samples in world file.

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double ptheta;

double grassX;
double grassY;
double initial_position_x;
double initial_position_y;
double initial_theta;
alpha_two::sheepState newmsg;

// X and Y position values of sheepDog1
double sheepDog1_x;
double sheepDog1_y;

// Toggle herding mode
bool herdingMode = false;

// Laser data for custom usage during herding mode
sensor_msgs::LaserScan laserData_msg;

void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index);
void initiateSheepHerding(nav_msgs::Odometry msg);
float CalculateAngularVelocity();

// Gets current angle between 0 and 360. Right is 0 (North)
void StageOdom_callback(nav_msgs::Odometry msg)
{
     ptheta = fmod((2*M_PI) + initial_theta + angles::normalize_angle_positive(asin(msg.pose.pose.orientation.z) * 2), 2*M_PI) * (180/M_PI);
     
     if(showDebug){
          ROS_INFO("PX: %f", msg.pose.pose.position.x);
     }
}

// Gets current x and y position relative to the world
// Sheep are limited to the boundaries set by the current
// position of the sheep dog.
void initiateSheepHerding(nav_msgs::Odometry msg){
    // Check current position of the sheep and compare with
    // broadcasted x and y position of the sheep dog and farmer.
    px = -msg.pose.pose.position.y;
    py = msg.pose.pose.position.x;
    
    ROS_INFO("PX: %f",sheepDog1_x);
    ROS_INFO("shPX: %f",px);
    ROS_INFO("shPY: %f",py);
    
    
    
    if(px < sheepDog1_y){
         ROS_INFO("This Sheep is behind enemy lines.");
         linear_x = 0;
         angular_z = (M_PI / 18) * 5; 
    }else{
         ROS_INFO("SAFE!");
         linear_x = 1;
         angular_z = 0;
    }
    ROS_INFO("diff: %f",px-sheepDog1_y);
}

void StageBasePose_callback(nav_msgs::Odometry msg)
{
  px = msg.pose.pose.position.y;
  py = -msg.pose.pose.position.x;
  if (showDebug){
      ROS_INFO("Current x position is: %f", px);
      ROS_INFO("Current y position is: %f", py);
  }
  
  if (herdingMode == true)
         initiateSheepHerding(msg);
}

void StageGrass_callback(alpha_two::grassState msg)
{
  //ROS_INFO("RECEIVED GRASS MESSAGE FROM: %d",msg.G_ID);
  if(newmsg.S_State == 1 && newmsg.grass_locked==msg.G_ID && msg.lockedBy != newmsg.S_ID)
  {
    newmsg.S_State = 0;
    if(showDebug)
        ROS_INFO("CHANGED STATE BACK TO LOOKING FOR GRASS");
  }
  else if(msg.G_State == 0 && newmsg.S_State == 0)
  {
    grassX = msg.x;
    grassY = msg.y;
    newmsg.S_State=1;
    newmsg.grass_locked = msg.G_ID;
    ROS_INFO("LOCKED GRASS: %d", msg.G_ID);
  }
  if(newmsg.grass_locked == msg.G_ID)
  {
    newmsg.grass_locked = 0;
    if(showDebug)
        ROS_INFO("STILL HAVE GRASS! YAY!");
  }
	if(newmsg.S_State == 1 && newmsg.grass_locked==msg.G_ID && msg.lockedBy != newmsg.S_ID){
		newmsg.S_State = 0;
	}else if(msg.G_State == 0 && newmsg.S_State == 0){
		grassX = msg.x;
		grassY = msg.y;
		newmsg.S_State=1;
                newmsg.grass_locked = msg.G_ID;
		
		if(showDebug)
		    ROS_INFO("LOCKED GRASS: %d",msg.G_ID);
	}
	if(newmsg.grass_locked == msg.G_ID){
		//newmsg.grass_locked = 0;
		if (showDebug)
		    ROS_INFO("STILL HAVE GRASS! YAY!");
	}
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
  if(showDebug)
    ROS_INFO("------------------------ Intensity ----------------------------");

  for(unsigned int i = 0; i < msg.intensities.size(); ++i){
    // Either 1 or 0 is returned. 1 if an object is view 0 otherwise.
    double curRange = msg.intensities[i];
    if(showDebug)
      ROS_INFO("Current intensity is    : %f", curRange);
  }

  if(showDebug)
    ROS_INFO("-------------------------- Range -----------------------------");

  int current_lowest_index = 0;
  double smallest_range = msg.ranges[current_lowest_index];
  // Iterate through LaserScan messages and find the smallest range
  for(unsigned int i = 1; i < msg.ranges.size(); ++i)
  {
    if(showDebug)
      ROS_INFO("Current range is    : %f", msg.ranges[i]);

    if(msg.ranges[current_lowest_index] > msg.ranges[i])
    {
      current_lowest_index = i;
      smallest_range = msg.ranges[current_lowest_index];
    }
  }  
  
  if (!herdingMode){
      collisionAvoidance(smallest_range, msg, current_lowest_index);
  }else{
       // Store laser data as global variable
       // This allows custom usage of the laser during herding mode.
       laserData_msg = msg;
  }

  // Publish the message
  RobotNode_stage_pub.publish(RobotNode_cmdvel);
  if(showDebug)
    ROS_INFO("-------------------------- END -----------------------------");
}

void sheepDog1_callback(alpha_two::sheepDogState msg){
    sheepDog1_x = msg.x;
    sheepDog1_y = msg.y;
    if(showDebug){
        ROS_INFO("SheepDog1 position x: %d",sheepDog1_x);
        ROS_INFO("SheepDog1 position y: %d",sheepDog1_y);
    }
}

// This function prevents robot(sheep) from colliding with stage objects.
void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index){
  if(newmsg.S_State!=1){  
	  if(showDebug)
	    ROS_INFO("Lowest index: %f", current_lowest_index);

	      // If the lowest range is less than 1.5 in length, the robot will begin
	      // rotating to attempt to avoid the obstacle
    	  if(smallest_range < 1.4) //between 0.8 and 1.5 exclusive
	      {
            // We are getting close to an obstacle, start turning
	        if(showDebug)
	          ROS_INFO("Collision at beam: %d | Range: %f", current_lowest_index, smallest_range);

            // Slow the robot down to 0.1m/s
            linear_x = 0.5;

            // Decide whether to turn anti-clockwise or clockwise depending on which
            // side of the robot is closest to the object
            if(current_lowest_index < SAMPLE_NUMBER/2) // sample_number is the number of beams
            {
                angular_z = (M_PI / 18) * 5; //anti-clockwise
            }else{
                angular_z = -(M_PI / 18) * 5; //clockwise
          }
          
          if(smallest_range <= 0.8) //we are really close to colliding, stop moving forward
          {
            linear_x = 0;
          }
          
      }else{
          // If no potential collisions are detected, move forward normally
          linear_x = 1;
          angular_z = 0;
      }
  }
  else if(newmsg.S_State==1)
  {
    ROS_INFO("GOING TO GRASS");
    linear_x = float(1) * atan2(grassX-px,grassY-py);
    angular_z = CalculateAngularVelocity();
    ROS_INFO("linear velocity: %f   angular velocity: %f", linear_x, angular_z);
  }
}

float CalculateAngularVelocity() {

	float deltaX = grassX - px;
	float deltaY = grassY-py;

	float changeInAngle;

	//Angle from origin
	float angle = atan(deltaY / deltaX);
	angle = 180 * angle / M_PI;

	//Find the quadrant to work out difference
	if (deltaX >= 0 && deltaY >= 0) { // right and up
		changeInAngle = angle - ptheta;
	} else if (deltaX >= 0 && deltaY <= 0) { // right and down
		changeInAngle = angle - ptheta;
	} else if (deltaX <= 0 && deltaY >= 0) { // left and up
		changeInAngle = 180 - ptheta + angle;
	} else if (deltaX <= 0 && deltaY <= 0) { // left and down
		changeInAngle = angle - 180 - ptheta;
	}

	//Make sure angle is between -360 and 360
	changeInAngle = fmodf(changeInAngle, 360.0);

	float angularZ=0;
	if (changeInAngle <= 2 && changeInAngle > 359) {
		angularZ = 0;
	} else if (changeInAngle >= 2 && changeInAngle <= 180) {
		angularZ = 0.5;
	} else if (changeInAngle <= 359 && changeInAngle > 180) {
		angularZ = -0.5;
	} else if (changeInAngle < 2 && changeInAngle >= -2) {
		angularZ = 0;
	} else if (changeInAngle <= -2 && changeInAngle >= -180) {
		angularZ = -0.5;
	} else if (changeInAngle <= -180) {
		angularZ = 0.5;
	} 
	ROS_INFO("grassX: %f   grassY: %f  px:%f   py:%f changeInAngle: %f",grassX,grassY, px, py, changeInAngle);
	return angularZ;
	
}

int main(int argc, char** argv){
  initial_position_x = atoi(argv[2]);
  initial_position_y = atoi(argv[3]);
  initial_theta = atoi(argv[4]) / (180/M_PI);
  std::stringstream rName;
  rName.str("");
  rName << "RobotNode" << argv[1];
  // You must call ros::init() first of all. ros::init() function needs to see argc and argv
  // The third argument is the name of the node
  ros::init(argc, argv, rName.str());
  ros::NodeHandle n;

  // Advertise new velocity of sheep to stageros
  rName.str("");
  rName << "robot_" << argv[1]<<"/cmd_vel";
  RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>(rName.str(),1000);

  // Obtain laser data from stageros  
  rName.str("");
  rName << "robot_" << argv[1]<<"/base_scan";
  ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>(rName.str(),1000,StageLaser_callback);
  
  rName.str("");
  rName << "robot_" << argv[1]<<"/odom";
  ros::Subscriber StageOdom_sub = n.subscribe<nav_msgs::Odometry>(rName.str(),1000,StageOdom_callback);
  
  rName.str("");
  rName << "robot_" << argv[1]<<"/base_pose_ground_truth";
  ros::Subscriber StageOdom_base_pose_sub = n.subscribe<nav_msgs::Odometry>(rName.str(),1000,StageBasePose_callback);

  // Listen to custom location messages from SheepDog1
  ros::Subscriber sheepDog1_position = n.subscribe<alpha_two::sheepDogState>("/sheepDog1_msg",1000,sheepDog1_callback);

  //ros::Subscriber grassNode_sub = n.subscribe<alpha_two::grassState>("Grass_msg", 1000, StageGrass_callback); 

  //ros::Publisher sheepNode_state = n.advertise<alpha_two::sheepState>("Sheep_msg",1000);

  newmsg.S_State = 0;
  newmsg.S_ID = atoi(argv[1]);
  newmsg.health = 100;
  newmsg.grass_locked = 0;
  
  int count = 0;
  ros::Rate r(10); // 10 cycles per second

  while(n.ok())
  {
    RobotNode_cmdvel.linear.x = linear_x;
    RobotNode_cmdvel.angular.z = angular_z;
    // Publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
    // sheepNode_state.publish(newmsg);
    ros::spinOnce(); //Must Have this statement in the program
    r.sleep();
    ++count;
  }
}

