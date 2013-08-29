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
double growth_rate;
//bool isGrassLiving;
double grass_hp; //variable for determining when the grass dies
double grass_age; //growth status. ranges from 0-3. also used for rotation speed.

//for HP recovery : if soil condtion is above 30. 
//HP will decrease if soil condition is below 30.


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
  //float onex = msg.pose.pose.orientation.x;
  //float oney = msg.pose.pose.orientation.y;
  //printf("px = %f   py = %f x = %f  y = %f",px, py, onex, oney);
}

void StageSheep_callback(alpha_two::sheepState msg)
{
  //ROS_INFO("RECEIVED Sheep MESSAGE FROM: %d",msg.S_ID);
  //ROS_INFO("Sheep message: %d, %d, %d, %d", msg.S_State, msg.S_ID, msg.health, msg.grass_locked);
  //if(msg.S_State==1 && msg.grass_locked==grass_state.G_ID && grass_state.G_State == 0){
  //  grass_state.G_State=1;
  //  grass_state.lockedBy = msg.S_ID;
  //  ROS_INFO("LOCKED by SHEEP: %d",msg.S_ID);
  //}
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
    if(grass_state.lockedBy == msg.S_State) //grass is locked to this sheep
    {
      if(msg.grass_locked != grass_state.G_ID) //sheep is not locked to this grass
      {
        //this may occur from rechecking which grass is closest
        grass_state.G_State = 0;
        grass_state.lockedBy = 0;
      }
    }
    if(msg.S_State == 2) //sheep is eating this grass
    {
      grass_hp -= 1; //grass is being eaten
      if(grass_hp <= 1) // Check if grass is eaten
      {
        grass_state.G_State = 2; //grass is now eaten
        grass_state.lockedBy = 0; //grass is no longer locked to a sheep
      }
    }
  }
}

void grass_update(double growth_rate){
 
  if(grass_hp == 0)
  {
    grass_age = 0;
  }
  // growing in good weather for grass HP
  // the ranges for growth_rate need to be adjusted when the soil values from farm.cpp change.
  else if(growth_rate > 0.07 && grass_hp + growth_rate < 20)
  {
    grass_hp += growth_rate;
  }
  // growing in bad weather
  else if(growth_rate <= 0.07 && grass_hp - growth_rate >= 0)
  {
    grass_hp -= growth_rate;
  }
  // growing in good weather for grass HP
  if (growth_rate > 0.07  &&  grass_age+growth_rate < 3){
    grass_age += growth_rate;
  }


}


void FarmNode_callback(alpha_two::farmState msg){
  //ROS_INFO("Farm 1: %d",msg.f1_soil_condition);
  //ROS_INFO("Farm 2: %d",msg.f2_soil_condition);
  //ROS_INFO("Farm 3: %d",msg.f3_soil_condition);
  //ROS_INFO("Farm 4: %d",msg.f4_soil_condition);



  if(initial_position_x>0 && initial_position_y>0){ //Field 1
                                                        // always consider the values coming in from farm.cpp.
    growth_rate = (double(msg.f1_soil_condition)/1000); // setting it to divide by 1000 gives us 0 ~ 0.2
    grass_update(growth_rate);
  }

  else if(initial_position_x>0 && initial_position_y<0){ //Field 2
    growth_rate = (double(msg.f2_soil_condition)/1000);
    grass_update(growth_rate);
  }

  else if(initial_position_x<0 && initial_position_y<0){ //Field 3
    growth_rate = (double(msg.f3_soil_condition)/1000);
    grass_update(growth_rate);
  }

  else if(initial_position_x<0 && initial_position_y>0){ //Field 4
    growth_rate = (double(msg.f4_soil_condition)/1000);
    grass_update(growth_rate);
  }

  //growth rate will be between 0~1
  //soil_condition ranges from 0~100
}

void StageLaser_callback(sensor_msgs::LaserScan msg){
  //This is the callback function to process laser scan messages
  //you can access the range data from msg.ranges[i]. i = sample number
}



int main(int argc, char **argv){
  //   growth_rate = 1;
  //initialize robot parameters
  //Initial pose. This is same as the pose that you used in the world file to set the robot pose.
  //initial_theta = M_PI/2.0;

  grass_hp = 10.0; //intial value for grass's hit point

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

  ///messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;
  RobotNode_cmdvel.angular.z = 2.0; // this dynamically updates the age of our grass; spinning it faster the older it is.
  
  grass_state.G_State = 0;
  grass_state.G_ID = atoi(argv[1]);
  initial_position_x = atoi(argv[2]);
  initial_position_y = atoi(argv[3]);
  while (ros::ok()){
    grass_state.x = initial_position_x;
    grass_state.y = initial_position_y;

    //publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
    grassNode_pub.publish(grass_state);
    ros::spinOnce();
    
    //prints for debugging
    //printf("Grass HP is: %f \n", grass_hp);
    //printf("GROWTH RATE: %f\n", growth_rate);
    //printf("Grass age is: %f \n", grass_age);
    
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

