#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/grassState.h"
#include "alpha_two/sheepState.h"
#include "alpha_two/farmState.h"
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

alpha_two::grassState grass_state;
int state;
double initial_position_x;
double initial_position_y;
double initial_theta;
double soil_quality;
double grass_grows = 0.25;
double grass_dies = 0.25;
double grass_hp; //variable for determining when the grass dies
// grass age is not redundant (since we want to just use the sheep's grass eating speed of 1 per tick, and using the grass_hp's reaction to that sounds good).
// double grass_age; //growth status. ranges from 0-3. also used for rotation speed.


struct instruction_struct{
  int step_count; //how many times do we execute this step before we move to the next step
  double linear_x; //linear velocity in m/s
  double angular_z; //angular velocity
  int next_step; //which step do we move to when we finish this current step
};

void StageOdom_callback(nav_msgs::Odometry msg){
  //This is the call back function to process odometry messages coming from Stage.
  px = 5 + msg.pose.pose.position.x;
  py =10 + msg.pose.pose.position.y;

}

void StageSheep_callback(alpha_two::sheepState msg)
{

  if(grass_state.G_State == 0) //free to be locked by a sheep
  {
    if(msg.S_State == 1 && msg.grass_locked == grass_state.G_ID) //sheep is locked to this grass
    {
      grass_state.G_State = 1;
      grass_state.lockedBy = msg.S_ID;
      ROS_INFO("LOCKED by SHEEP: %d",msg.S_ID);
    }
  }
  else if(grass_state.G_State == 1) //locked by a sheep
  {
    if(msg.S_State == 2) //sheep is eating this grass
    {

      // grass eating speed has been changed (from 1). The grass still dies, but it will still spin slowly.
      grass_hp -= 0.5; //grass is being eaten
      if(grass_hp <= 0.5) // Check if grass is eaten
      {
        grass_state.G_State = 2; //grass is now eaten
        grass_state.lockedBy = 0; //grass is no longer locked to a sheep
      }
    }
    if(grass_state.lockedBy == msg.S_ID) //sheep is no longer locked to this grass
    {
      if(msg.grass_locked != grass_state.G_ID)
      {
        grass_state.G_State = 0; //grass is available to be eaten
        grass_state.lockedBy = 0; //grass is no longer locked to a sheep
      }
    }
  }
}

void grass_update(double soil_quality){

  if(grass_hp <= 1) // if the hp is lower than 1 (by being eaten or by weather) then it dies (G_State = 2)
  {
    grass_state.G_State = 2;  // grass is dead (eaten).
  }

  if (grass_hp > 3 && grass_state.G_State == 2) // if the grass regains enough HP to start spinning and be eaten again
  {
    grass_state.G_State = 0; // the grass can be eaten by sheeps again.
  }

  if ( grass_state.G_State == 0)
  {
       // growing in good weather for grass HP
    if(soil_quality > 0.05 && grass_hp + grass_grows < 4)  // the second condition ensures that the HP never goes up too high to not show a quick response..
    {
    // instead of adding soil_quality, now we're adding a contant number.
      grass_hp += grass_grows;
    }
  // dying in bad weather
      else if(soil_quality <= 0.05 && grass_hp - grass_dies >= 0) // the second condition ensures that the HP never goes negative.
    {
      grass_hp -= grass_dies;
    }
  }
}


void FarmNode_callback(alpha_two::farmState msg){

  // Each grass receives the soil value coresponding to its position on the farm.
  // always consider the values coming in from farm.cpp.
  // as of 29 Aug, 5:30pm, it sends soil values from 0 to 100.

  if(initial_position_x>0 && initial_position_y>0){ //Field 1
    soil_quality = (double(msg.f1_soil_condition)/1000); // setting it to divide by 1000 gives us 0 ~ 0.1
    grass_update(soil_quality);
  }

  else if(initial_position_x>0 && initial_position_y<0){ //Field 2
    soil_quality = (double(msg.f2_soil_condition)/1000);
    grass_update(soil_quality);
  }

  else if(initial_position_x<0 && initial_position_y<0){ //Field 3
    soil_quality = (double(msg.f3_soil_condition)/1000);
    grass_update(soil_quality);
  }

  else if(initial_position_x<0 && initial_position_y>0){ //Field 4
    soil_quality = (double(msg.f4_soil_condition)/1000);
    grass_update(soil_quality);
  }

}

void StageLaser_callback(sensor_msgs::LaserScan msg){
  //This is the callback function to process laser scan messages
  //you can access the range data from msg.ranges[i]. i = sample number
}



int main(int argc, char **argv){

  //initialize robot parameters
  //Initial pose. This is same as the pose that you used in the world file to set the robot pose.
  //initial_theta = M_PI/2.0;

  grass_hp = 2.1; //intial value for grass's hit point. A good value that isn't too high not to be immediately affected by the climate.

  //Initial velocity
  linear_x = 0;
  angular_z = 0;

  px = 26;
  py = 0;

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

  ros::Subscriber FarmNode_sub = n.subscribe<alpha_two::farmState>("farm_msg",1000, FarmNode_callback);

  rName.str("");
  rName << "robot_" << argv[1]<<"/base_scan";
  ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>(rName.str(),1000,StageLaser_callback);

  ros::Rate loop_rate(10);

  //a count of howmany messages we have sent
  int count = 0;

  state = atoi(argv[1]);

  //messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;


  grass_state.G_State = 0;
  grass_state.G_ID = atoi(argv[1]);
  initial_position_x = atoi(argv[2]);
  initial_position_y = atoi(argv[3]);

  // Work out what quadrant this grass is in
  if(initial_position_x > 0 && initial_position_y > 0) //quadrant 1
  {
    grass_state.quadrant = 1;
  }
  else if(initial_position_x > 0 && initial_position_y < 0) //quadrant 2
  {
    grass_state.quadrant = 2;
  }
  else if(initial_position_x < 0 && initial_position_y < 0) //quadrant 3
  {
    grass_state.quadrant = 3;
  }
  else if(initial_position_x < 0 && initial_position_y > 0) //quadrant 4
  {
    grass_state.quadrant = 4;
  }

  while (ros::ok()){
    grass_state.x = initial_position_x;
    grass_state.y = initial_position_y;

    // This line of code belongs here in the continuously running while loop, not outside of it.
    RobotNode_cmdvel.angular.z = grass_hp; // this dynamically updates the age of our grass; spinning it faster the older it is.

    //publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
    grassNode_pub.publish(grass_state);
    ros::spinOnce();

    //prints for debugging
    printf("Grass HP is: %f \n", grass_hp);



    loop_rate.sleep();
    ++count;
  }
  return 0;
}

