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
ros::Publisher SheepNode_state;
geometry_msgs::Twist RobotNode_cmdvel;

bool debug = false; //Show/hide ROS log messages
#define SAMPLE_NUMBER 10 // represents number of samples in world file.

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double ptheta;

//position of closest grass and distance from sheep
double grass_x;
double grass_y;
double grass_distance = 99999; //we need to init it for comparison purposes

//initial position of robot
double initial_position_x;
double initial_position_y;
double initial_theta;

//used for advertising what the sheep is currently doing
alpha_two::sheepState sheep_message; //please do not call variables "newmsg" ever again

// X and Y position values of sheepDog1
double sheepDog1_x;
double sheepDog1_y;

// Toggle herding mode
bool is_being_herded = false;

// Laser data for custom usage during herding mode
sensor_msgs::LaserScan laserData_msg;

void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index);
void initiateSheepHerding(nav_msgs::Odometry msg);
float CalculateAngularVelocity();
float CalculateLinearVelocity();

// Gets current angle between 0 and 360. Right is 0 (North)
void StageOdom_callback(nav_msgs::Odometry msg)
{
   ptheta = fmod((2*M_PI) + initial_theta + angles::normalize_angle_positive(asin(msg.pose.pose.orientation.z) * 2), 2*M_PI) * (180/M_PI);

   if(debug)
     ROS_INFO("PX: %f", msg.pose.pose.position.x);
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

  if(px < sheepDog1_x)
  {
    ROS_INFO("This Sheep is behind enemy lines.");
    linear_x = 0;
    angular_z = (M_PI / 18) * 5;
  }
  else
  {
    ROS_INFO("SAFE!");
    linear_x = 1;
    angular_z = 0;
  }
  ROS_INFO("diff: %f",px-sheepDog1_x);
}

void StageBasePose_callback(nav_msgs::Odometry msg)
{
  px = msg.pose.pose.position.y;
  py = -msg.pose.pose.position.x;
  if (debug)
  {
    ROS_INFO("Current x position is: %f", px);
    ROS_INFO("Current y position is: %f", py);
  }
  if (is_being_herded == true)
    initiateSheepHerding(msg);
}

/*
 * We receive one of these grass_state msgs from every grass
 * Thus, use a lot of global variables to keep track of information
 */
void StageGrass_callback(alpha_two::grassState msg)
{
  ROS_INFO("RECEIVED GRASS MESSAGE FROM: %d",msg.G_ID);
  /*
   * Check if this piece of grass is currently locked by any
   * other sheep. If grass is already locked, ignore this piece
   * of grass because sheep don't share!
   */
  if(msg.lockedBy != sheep_message.S_ID && msg.lockedBy != 0)
  {
    return;
  }
  
  /*
   * Check if sheep is currently locked onto a piece of grass, 
   * if it is, ignore all other grass messages
   */
  if(sheep_message.S_State == 1)
  {
    return;
  } 

  /*
   * Check if sheep is currently locked onto this piece of grass
   * If it is, don't change anything, sheep will keep walking
   * towards it
   */
  if(sheep_message.grass_locked == msg.G_ID)
  {
    if(debug)
      ROS_INFO("Locked onto grass: %d", msg.G_ID);
    //go towards grass
    angular_z = CalculateAngularVelocity();
    //slow down if sheep gets close to the grass
    linear_x = CalculateLinearVelocity();
    return;
  }

  /*
   * If sheep is currently looking for a piece of grass, calculate
   * the distance between itself and the grass. If it's closer
   * to the piece of grass, save the grass coordinates
   */
  if(sheep_message.S_State == 0 && msg.G_State == 0)
  {
    double distance_x = px - msg.x;
    double distance_y = py - msg.y;
    double distance = sqrt(pow(distance_x, 2.0) + pow(distance_y, 2.0));
    if(distance < grass_distance)
    {
      sheep_message.S_State = 1;
      sheep_message.grass_locked = msg.G_ID;
      ROS_INFO("Attempting to lock onto grass");
      grass_distance = distance;
      grass_x = msg.x;
      grass_y = msg.y;
    }
  }

  // Not sure what the rest of this stuff does?
  if(sheep_message.S_State == 1 && sheep_message.grass_locked==msg.G_ID && msg.lockedBy != sheep_message.S_ID)
  {
    sheep_message.S_State = 0;
    if(debug)
      ROS_INFO("CHANGED STATE BACK TO LOOKING FOR GRASS");

  }
}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
  if(debug)
    ROS_INFO("------------------------ Intensity ----------------------------");

  for(unsigned int i = 0; i < msg.intensities.size(); ++i){
    // Either 1 or 0 is returned. 1 if an object is view 0 otherwise.
    double curRange = msg.intensities[i];
    if(debug)
      ROS_INFO("Current intensity is    : %f", curRange);
  }

  if(debug)
    ROS_INFO("-------------------------- Range -----------------------------");

  int current_lowest_index = 0;
  double smallest_range = msg.ranges[current_lowest_index];
  // Iterate through LaserScan messages and find the smallest range
  for(unsigned int i = 1; i < msg.ranges.size(); ++i)
  {
    if(debug)
      ROS_INFO("Current range is    : %f", msg.ranges[i]);

    if(msg.ranges[current_lowest_index] > msg.ranges[i])
    {
      current_lowest_index = i;
      smallest_range = msg.ranges[current_lowest_index];
    }
  }

  if(!is_being_herded)
  {
    //go towards grass
    angular_z = CalculateAngularVelocity();
    //slow down if sheep gets close to the grass
    linear_x = CalculateLinearVelocity();
    // if theres a wall or sheep in the way, avoid it
    collisionAvoidance(smallest_range, msg, current_lowest_index);
  }
  else
  {
    // Store laser data as global variable
    // This allows custom usage of the laser during herding mode.
    laserData_msg = msg;
  }
  if(debug)
    ROS_INFO("-------------------------- END -----------------------------");
}

void sheepDog1_callback(alpha_two::sheepDogState msg){
  sheepDog1_x = msg.x;
  sheepDog1_y = msg.y;
  if(debug){
    ROS_INFO("SheepDog1 position x: %f",sheepDog1_x);
    ROS_INFO("SheepDog1 position y: %f",sheepDog1_y);
  }
}

// This function prevents robot(sheep) from colliding with stage objects.
void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index) {
  if(sheep_message.S_State!=1){
    if(debug)
      ROS_INFO("Lowest index: %d", current_lowest_index);

    // If the lowest range is less than 1.5 in length, the robot will begin
    // rotating to attempt to avoid the obstacle
    if(smallest_range < 1.4) //between 0.8 and 1.5 exclusive
    {
      // We are getting close to an obstacle, start turning
      if(debug)
        ROS_INFO("Collision at beam: %d | Range: %f", current_lowest_index, smallest_range);

      // Slow the robot down to 0.1m/s
      linear_x = 0.5;

      // Decide whether to turn anti-clockwise or clockwise depending on which
      // side of the robot is closest to the object
      if(current_lowest_index < SAMPLE_NUMBER/2) // sample_number is the number of beams
        angular_z = (M_PI / 18) * 5; //anti-clockwise
      else
        angular_z = -(M_PI / 18) * 5; //clockwise

      if(smallest_range <= 0.8) //we are really close to colliding, stop moving forward
        linear_x = 0;
    }
    else
    {
        // If no potential collisions are detected, move forward normally
        linear_x = 1;
        angular_z = 0;
    }
  }
  else if(sheep_message.S_State==1)
  {
    ROS_INFO("GOING TO GRASS");
    linear_x = float(1) * atan2(grass_x-px,grass_y-py);
    angular_z = CalculateAngularVelocity();
    ROS_INFO("linear velocity: %f   angular velocity: %f", linear_x, angular_z);
  }
}

float CalculateAngularVelocity() {

        float deltaX = grass_x - px;
        float deltaY = grass_y-py;

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
        //ROS_INFO("grass_x: %f   grass_y: %f  px:%f   py:%f changeInAngle: %f",grass_x,grass_y, px, py, changeInAngle);
        return angularZ;

}

float CalculateLinearVelocity()
{
  linear_x = sqrt(grass_distance);
  return linear_x;
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

  // Advertise what the sheep is currently doing
  SheepNode_state = n.advertise<alpha_two::sheepState>("/Sheep_msg",1000);

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

  ros::Subscriber grassNode_sub = n.subscribe<alpha_two::grassState>("Grass_msg", 1000, StageGrass_callback);

  ros::Publisher sheepNode_state = n.advertise<alpha_two::sheepState>("Sheep_msg",1000);

  sheep_message.S_State = 0;
  sheep_message.S_ID = atoi(argv[1]);
  sheep_message.health = 100;
  sheep_message.grass_locked = 0;

  int count = 0;
  ros::Rate r(10); // 10 cycles per second

  while(n.ok())
  {
    RobotNode_cmdvel.linear.x = linear_x;
    RobotNode_cmdvel.angular.z = angular_z;
    // Publish the messages
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
    SheepNode_state.publish(sheep_message);
    ros::spinOnce(); //Must Have this statement in the program
    grass_distance = 99999; //reset grass_distance for next tick
    r.sleep();
    ++count;
  }
}

