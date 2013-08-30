#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/grassState.h"
#include "alpha_two/sheepState.h"
#include "alpha_two/sheepDogState.h"
#include "alpha_two/herdingBar.h"
#include <sstream>
#include "math.h"
#include <time.h>

using namespace std;
// Global variables and objects
ros::Publisher RobotNode_stage_pub;
ros::Publisher RobotNode_stage_poop;
ros::Publisher SheepNode_state;
geometry_msgs::Twist RobotNode_cmdvel;
geometry_msgs::Twist poopNode_cmdvel;
geometry_msgs::Twist ill_cmdvel;

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

//quadrant sheep is currently in
int quadrant;

//initial position of robot
double initial_position_x;
double initial_position_y;
double initial_theta;

//used for advertising what the sheep is currently doing
alpha_two::sheepState sheep_message; //please do not call variables "newmsg" ever again

// X and Y position values of sheepDog
double sheepDog_x;
double sheepDog_y;

// Toggle herding mode
bool is_being_herded = false;

// Laser data for custom usage during herding mode
sensor_msgs::LaserScan laserData_msg;

void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index);
void initiateSheepHerding(nav_msgs::Odometry msg);
void CalculateGrassDistance();
void reset_sheep();
void calculate_quadrant();
float CalculateAngularVelocity();
float CalculateLinearVelocity();

// Gets current angle between 0 and 360. Right is 0 (North)
void StageOdom_callback(nav_msgs::Odometry msg)
{
  ptheta = fmod((2*M_PI) + initial_theta + angles::normalize_angle_positive(asin(msg.pose.pose.orientation.z) * 2), 2*M_PI) * (180/M_PI);
  if(debug)
    ROS_INFO("Current theta: %f", ptheta);
}

/* Gets current x and y position relative to the world
 * Sheep are limited to the boundaries set by the current
 * position of the sheep dog.
 */
void initiateSheepHerding(nav_msgs::Odometry msg)
{
  // Check current position of the sheep and compare with
  // broadcasted x and y position of the sheep dog and farmer.
  px = -msg.pose.pose.position.y;
  py = msg.pose.pose.position.x;

  ROS_INFO("PX: %f",sheepDog_x);
  ROS_INFO("shPX: %f",px);
  ROS_INFO("shPY: %f",py);

  if(px < sheepDog_x)
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
  ROS_INFO("diff: %f",px-sheepDog_x);
}

void StageBasePose_callback(nav_msgs::Odometry msg)
{
  //Stage base pose north is to the right. Transform it so upwards is north
  px = -msg.pose.pose.position.y;
  py = msg.pose.pose.position.x;
  //if(is_being_herded == true)
    //initiateSheepHerding(msg);
}


bool receivedHerdingMsg = false;
void HerdingBar_callback(alpha_two::herdingBar msg)
{ 
  // Multiple messages are being received from Gates node
  // boolean below makes sure is_being_herded is set after just the first message.
  //if (!receivedHerdingMsg)
  //{
    if (msg.herdingMode == 0)
    {
      is_being_herded = true;
      ROS_INFO("SHEEP BEING HERDED!");
      sheep_message.S_State = 3;
      reset_sheep();
      
    }
		else if (msg.herdingMode == 2)
    {
      is_being_herded = false;
      ROS_INFO("HERDING STOPPED!");
      sheep_message.S_State = 0;
      reset_sheep();
    }
    //receivedHerdingMsg = true;
  //}

}

/*
 * We receive one of these grass_state msgs from every grass
 * Thus, use a lot of global variables to keep track of information
 */
void StageGrass_callback(alpha_two::grassState msg)
{
  if(debug)
    ROS_INFO("Received grass message: %d, %d, %d, %d, %d", msg.G_State, msg.G_ID, msg.x, msg.y, msg.lockedBy);

  if(quadrant != msg.quadrant) // If grass is not in the same field, ignore it
    return;

  if(is_being_herded)
  {
  	return;
	}

  // Continuously re-check which grass is closest if we are not currently eating grass
  if(sheep_message.S_State == 0 || sheep_message.S_State == 1)
  {
    if(msg.G_State == 0) //grass is not currently locked by a sheep
    {
      if(debug)
        ROS_INFO("Calculating distance to grass: %d", msg.G_ID);
      double distance_x = px - msg.x;
      double distance_y = py - msg.y;
      double distance = sqrt(pow(distance_x, 2.0) + pow(distance_y, 2.0));
      if(distance < grass_distance) //this is the closest grass so far
      {
        /*
         * Attempt to lock onto this piece of grass. We will know if we are
         * successful during the next grass tick when the grass responds with
         * which sheep that grass is locked to.
         */
        if(debug)
          ROS_INFO("Attempting to lock onto grass: %d", msg.G_ID);
        sheep_message.S_State = 1;
        sheep_message.grass_locked = msg.G_ID;
        grass_distance = distance;
        grass_x = msg.x;
        grass_y = msg.y;
      }
      if(debug)
        ROS_INFO("Distance to grass: %d is %f", msg.G_ID, grass_distance);
    }
  }
  else if(msg.G_State == 2 || msg.G_State == 3) //grass is eaten or dead
  {
    if(sheep_message.grass_locked == msg.G_ID) //sheep was locked to this grass
    {
      /*
       * Since the grass is now eaten or dead, the sheep no longer wants to eat
       * this piece of grass. Shift back to searching for a piece of grass
       */
      ROS_INFO("Can't eat this grass anymore. Finding a new piece of grass");
      reset_sheep();
      return;
    }
  }

  if(sheep_message.S_State == 0) //sheep is not locked onto anything
  {

  }
  else if(sheep_message.S_State == 1) //sheep is locked onto a grass
  {
    if(sheep_message.grass_locked == msg.G_ID) //sheep is locked to this grass
    {
      if(msg.lockedBy == sheep_message.S_ID) //grass is locked to this sheep
      {
        // Sheep and grass are locked to each other, move towards grass
        angular_z = CalculateAngularVelocity(); //turn towards grass
        linear_x = CalculateLinearVelocity(); //calculate speed based on distance to grass
      }
      else //sheep was locked to the grass, but the grass was not locked to this sheep
      {
        if(msg.G_State == 0) //sheep locked to grass, grass must not yet be updated
        {
          return;
        }
        else if(msg.G_State == 1) //sheep locked to grass, grass not locked to this sheep and is locked by another sheep
        {
          //reset the sheep back to searching mode
          reset_sheep();
          if(debug)
            ROS_INFO("Attempt to lock onto grass failed. Going back to searching");
          return;
        }
      }
    }
    else //sheep is not locked a different grass, ignore this grass
    {
      return;
    }
  }
  else if(sheep_message.S_State == 2) //sheep is eating grass
  {
    ROS_INFO("Eating grass");
    angular_z = 0;
    linear_x = 0;
  }
}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
  // Iterate through each LaserScan messages to find the smallest range
  int current_lowest_index = 0;
  double smallest_range = msg.ranges[current_lowest_index];
  for(unsigned int i = 1; i < msg.ranges.size(); ++i)
  {
    if(msg.ranges[current_lowest_index] > msg.ranges[i])
    {
      current_lowest_index = i;
      smallest_range = msg.ranges[current_lowest_index];
    }
  }

  if(is_being_herded) //if sheep is currently being herded
  {
    // Store LaserScan data as global variable
    // This allows usage of the LaserScan data during herding mode
    // laserData_msg = msg;
    linear_x = 1;
    angular_z = 0;    
    ROS_INFO("INSIDE IF is_being_herded");
    collisionAvoidance(smallest_range, msg, current_lowest_index);
    
  }
  else //sheep is not being herded, free to move and eat grass
  {
    if(sheep_message.S_State == 0) //sheep is not locked onto anything, free movement
    {
      linear_x = 1;
      angular_z = 0;
    }
    else if(sheep_message.S_State == 1) //sheep is locked onto grass
    {
      ROS_INFO("Moving to grass");
      linear_x = CalculateLinearVelocity(); //slow down if sheep gets close to the grass
      angular_z = CalculateAngularVelocity(); //turn towards grass
      if(debug)
        ROS_INFO("linear velocity: %f   angular velocity: %f", linear_x, angular_z);
    }

    // Check for collisions with obstacles and attempt to avoid
    collisionAvoidance(smallest_range, msg, current_lowest_index);
    
    if(sheep_message.S_State == 2) //sheep is eating grass
    {
      ROS_INFO("Eating grass");
      linear_x = 0;
      angular_z = 0;
    }
  }
}


void reset_sheep()
{
  sheep_message.S_State = 0;
  sheep_message.grass_locked = 0;
  grass_distance = 99999;
  grass_x = 0;
  grass_y = 0;
}

void calculate_quadrant()
{
  if(px > 0 && py > 0) //quadrant 1
  {
    quadrant = 1;
  }
  else if(px > 0 && py < 0) //quadrant 2
  {
    quadrant = 2;
  }
  else if(px < 0 && py < 0) //quadrant 3
  {
    quadrant = 3;
  }
  else if(px < 0 && py > 0) //quadrant 4
  {
    quadrant = 4;
  }
}

// This function prevents robot(sheep) from colliding with stage objects.
void collisionAvoidance(double smallest_range, sensor_msgs::LaserScan msg, int current_lowest_index)
{
  //if(debug)
    ROS_INFO("Lowest index: %d", current_lowest_index);
  // If the lowest range is less than 1.5 in length, the robot will begin
  // rotating to attempt to avoid the obstacle
  if(smallest_range < 1.4) // We are getting close to an obstacle, start turning
  {
    //if(debug)
      ROS_INFO("Collision at beam: %d | Range: %f", current_lowest_index, smallest_range);
    if(smallest_range <= 0.8) //we are really close to colliding, stop moving forward
      linear_x = 0;
    else // Getting close, but not quite near, go forward slowly
      linear_x = 0.5;

    // Decide whether to turn anti-clockwise or clockwise depending on which
    // side of the robot is closest to the object
    if(current_lowest_index < SAMPLE_NUMBER/2) // sample_number is the number of beams
      angular_z = (M_PI / 18) * 5; //anti-clockwise
    else
      angular_z = -(M_PI / 18) * 5; //clockwise
  }
}

float CalculateAngularVelocity()
{
  float delta_x = grass_x - px;
  float delta_y = grass_y - py;
  float theta = atan(abs(delta_y) / abs(delta_x)) * (180 / M_PI); //calculate angle in degrees
  if(delta_x > 0 && delta_y > 0) //top left
  {
    theta = 0 + theta;
  }
  else if(delta_x < 0 && delta_y > 0) //top right
  {
    theta = 180 - theta;
  }
  else if(delta_x < 0 && delta_y < 0) //bottom left
  {
    theta = 180 + theta;
  }
  else if(delta_x > 0 && delta_y < 0) //bottom right
  {
    theta = 360 - theta;
  }
  // calculate whether its better to turn clockwise or anti-clockwise
  float difference = theta - ptheta;
  difference = fmod(difference + 360.0, 360.0);
  if(debug)
    ROS_INFO("Calculated theta: %f difference: %f delta_x: %f delta_y: %f", theta, difference, delta_x, delta_y);
  if(difference > 180)
    return -difference / (180/M_PI); //clockwise
  else
    return difference / (180/M_PI); //anti-clockwise
}

float CalculateLinearVelocity()
{
  CalculateGrassDistance(); //this updates grass_distance
  if(grass_distance < 0.8) //sheep is close to the grasss
  {
    sheep_message.S_State = 2; //sheep is now eating grass
    return 0;
  }
  return 0.5 * sqrt(grass_distance);
}

void CalculateGrassDistance()
{
  double distance_x = grass_x - px;
  double distance_y = grass_y - py;
  double distance = sqrt(pow(distance_x, 2.0) + pow(distance_y, 2.0));
  grass_distance = distance;
}

int main(int argc, char** argv)
{
  grass_distance = 99999;
  px = initial_position_x = atoi(argv[2]);
  py = initial_position_y = atoi(argv[3]);

  initial_theta = atoi(argv[4]) / (180/M_PI);
  std::stringstream rName;
  int poopNumber = atoi(argv[1]) + 1;
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

  ros::Subscriber grassNode_sub = n.subscribe<alpha_two::grassState>("Grass_msg", 1000, StageGrass_callback);

  ros::Publisher sheepNode_state = n.advertise<alpha_two::sheepState>("Sheep_msg",1000);

  ros::Subscriber herdingBar_state = n.subscribe<alpha_two::herdingBar>("herdingBar",1000, HerdingBar_callback);

  //Publisher for poop message
  rName.str("");
  rName << "robot_" << poopNumber <<"/cmd_vel";
  RobotNode_stage_poop = n.advertise<geometry_msgs::Twist>(rName.str(),1000);
  sheep_message.S_State = 0;
  sheep_message.S_ID = atoi(argv[1]);
  sheep_message.health = 100;
  sheep_message.grass_locked = 0;

  ill_cmdvel.linear.x = 0;
  ill_cmdvel.linear.y = 0;
  ill_cmdvel.angular.z = 0.5;

  //Poop related variables

  srand (time(NULL));

  int poopCount = 200 + rand()%300;
  poopNode_cmdvel.linear.x = 0;
  poopNode_cmdvel.angular.z = 0;
  bool pooping = false;

  bool ill = false;
  int respawn = 0;
  int count = 0;
  ros::Rate r(10); // 10 cycles per second
  int wellness = 1000;
  while(n.ok())
  {
    RobotNode_cmdvel.linear.x = linear_x;
    RobotNode_cmdvel.angular.z = angular_z;
    // Publish the messages

    if (is_being_herded)
    {
      RobotNode_stage_pub.publish(RobotNode_cmdvel);
			if (!pooping){
        RobotNode_stage_poop.publish(RobotNode_cmdvel);
      }
      else{
        RobotNode_stage_poop.publish(ill_cmdvel);
      }

    }else{
    if (ill){
      RobotNode_stage_pub.publish(ill_cmdvel);
      RobotNode_stage_poop.publish(ill_cmdvel);
      respawn++;
      if(respawn > 150){
        respawn = 0;
        ill = false;
        wellness = 1000;
      }
    }
    else if(wellness < 0 && sheep_message.S_State != 0){
      RobotNode_stage_pub.publish(RobotNode_cmdvel);
      if (!pooping){
        RobotNode_stage_poop.publish(RobotNode_cmdvel);
      }
      else{
        RobotNode_stage_poop.publish(ill_cmdvel);
      }

      wellness = 1000;
    }
    else if(wellness < 0 && sheep_message.S_State == 0){
      RobotNode_stage_pub.publish(ill_cmdvel);
      RobotNode_stage_poop.publish(ill_cmdvel);
      ill = true;
    }
    else{
      RobotNode_stage_pub.publish(RobotNode_cmdvel);
      if (!pooping){
        RobotNode_stage_poop.publish(RobotNode_cmdvel);
      }
      else{
        RobotNode_stage_poop.publish(ill_cmdvel);
      }
      wellness--;
    }


    }
  

    //ROS_INFO("Movement x: %f   y: %f", RobotNode_cmdvel.linear.x, RobotNode_cmdvel.angular.z);
    SheepNode_state.publish(sheep_message);
    ros::spinOnce(); //Must Have this statement in the program
    calculate_quadrant();
    //grass_distance = 99999; //reset grass_distance for next tick
    r.sleep();
    ++count;
    poopCount--;
    if (poopCount == -1){
      pooping = true;
    }
  }
}

