#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/grassState.h"
#include "alpha_two/sheepState.h"
#include "alpha_two/farmState.h"
#include "alpha_two/rainFall.h"
#include <sstream>
#include "math.h"

using namespace std;

alpha_two::farmState new_farm_msg;
alpha_two::rainFall new_rain_msg;

geometry_msgs::Twist rain_cmdvel;
geometry_msgs::Twist summer_cmdvel;
geometry_msgs::Twist winter_cmdvel;
geometry_msgs::Twist spring_cmdvel;
geometry_msgs::Twist autumn_cmdvel;

int dayCounter;
double px,py,rx,ry,smx,smy,wx,wy,ax,ay,spx,spy;
double sunlight;
// 1 - Spring; 2 - Summer; 3 - Autumn; 4 - Winter
int curSeason;
int rain_amount;
bool raining = false;
// Changes the weather conditions of the farm

/**
  ChangeWeather is the main function that is responsible for managing the weather for the farm Simulator.
  It keeps track of the dayCounter and determines what season it is, and also determines the rainfall
  according to the average rainfall figures that we obtained from the internet (later altered to show more realistic demo).
  It is also responsible for changing the soil condition of each field.
  author: Oriental Turtles
*/
void changeWeather(){
  
  dayCounter = dayCounter%732; // loops every 1 minute (approx)
  dayCounter++;
  //Rainfall is determined randomly according to the current season.
  //for demo purposes, values set are very dynamic. when set to realistic values
  //it's hard to see the difference visually

  //Spring
  if (dayCounter < 183)
  {        
    curSeason = 1;
    sunlight = 41;
  }

  //Summer
  else if(dayCounter < 366)
  {
    curSeason = 2;
    sunlight = 0;  // too much sunlight kills
  }
    
  //Autumn
  else if (dayCounter < 549)
  {        
    curSeason = 3;
    sunlight = 10;
  }

  //Winter
  else if (dayCounter < 732)
  {
  
    curSeason = 4;
    sunlight = 10; 
  }

  else
  {
    dayCounter = 0;
  }
  
  // the rain's amount is now decided here.
  if(dayCounter>0 && dayCounter<100)
  {
    raining = true;
    rain_amount = rand()%20;

  }
  else if (dayCounter >= 100 && dayCounter < 250)
  {
    raining = false;
    rain_amount = 0;
  }
  else if (dayCounter >= 250 && dayCounter < 380)
  {
    raining = true;
    rain_amount = rand()%20;
  }
  else if (dayCounter >= 380 && dayCounter < 520)
  {
    raining = false;
        rain_amount = 0;
  }
  else if (dayCounter >= 520 && dayCounter < 650)
  {
    raining = true;
    rain_amount = rand()%50;
  }
  else
  {
    raining = false;
    rain_amount = 0;
  }

  // Weather changes applied to each field.  
  // sums up all the factors that affect the soil quality and scales it.

  // field 1 = green, 2 = brown, 3 = yellow, 4 = light green
  new_farm_msg.f1_soil_condition = (50 + rain_amount + sunlight); // Always alive
  new_farm_msg.f2_soil_condition = (0 + rain_amount + sunlight); // Always dead, when raining in Spring
  new_farm_msg.f3_soil_condition = (10 + rain_amount + sunlight); // Alive in Spring and when raining in Winter
  new_farm_msg.f4_soil_condition = (40 + rain_amount + sunlight); // Alive in Spring, alive when raining during other seasons
}

/**
  ChangeWeatherMessage function changes the weather message that needs to be displayed according to the 
  curSeason variable that is set by the changeWeather function.   E.g when its summer, it moves the summer
  message down from under the messagehouse so its visible, and pulls any other messages back into the house
  if they are outside.
  Author: Oriental Turtles

*/
void changeWeatherMessage(){

  //Spring
  if(curSeason == 1){
    if(smy > -0.1 && smy > -0.1){
      summer_cmdvel.linear.y = 0;
    }
    else if(smy <= -3 && smy <= -3){
      summer_cmdvel.linear.y = 0.9;
    }
    
    if(spy > -0.1 && spy > -0.1){
      spring_cmdvel.linear.y = -0.9;
    }
    else if(spy <= -3 && spy <= -3){
      spring_cmdvel.linear.y = 0;
    }
   
    if(wy > -0.1 && wy > -0.1){
      winter_cmdvel.linear.y = 0;
    }
    else if(wy <= -3 && wy <= -3){
      winter_cmdvel.linear.y = 0.9;
    }

    if(ay > -0.1 && ay > -0.1){
      autumn_cmdvel.linear.y = 0;
    }
    else if(ay <= -3 && ay <= -3){
      autumn_cmdvel.linear.y = 0.9;
    }
  }

  // Summer
  else if(curSeason==2){
    if(smy > -0.1 && smy > -0.1){
      summer_cmdvel.linear.y = -0.9;
    }
    else if(smy <= -3 && smy <= -3){
      summer_cmdvel.linear.y = 0;
    }
    
    if(spy > -0.1 && spy > -0.1){
      spring_cmdvel.linear.y = 0;
    }
    else if(spy <= -3 && spy <= -3){
      spring_cmdvel.linear.y = 0.9;
    }
   
    if(wy > -0.1 && wy > -0.1){
      winter_cmdvel.linear.y = 0;
    }
    else if(wy <= -3 && wy <= -3){
      winter_cmdvel.linear.y = 0.9;
    }

    if(ay > -0.1 && ay > -0.1){
      autumn_cmdvel.linear.y = 0;
    }
    else if(ay <= -3 && ay <= -3){
      autumn_cmdvel.linear.y = 0.9;
    } 
  }

  //Autumn
  else if(curSeason == 3){
    if(smy > -0.1 && smy > -0.1){
      summer_cmdvel.linear.y = 0;
    }
    else if(smy <= -3 && smy <= -3){
      summer_cmdvel.linear.y = 0.9;
    }
    
    if(spy > -0.1 && spy > -0.1){
      spring_cmdvel.linear.y = 0;
    }
    else if(spy <= -3 && spy <= -3){
      spring_cmdvel.linear.y = 0.9;
    }
   
    if(wy > -0.1 && wy > -0.1){
      winter_cmdvel.linear.y = 0;
    }
    else if(wy <= -3 && wy <= -3){
      winter_cmdvel.linear.y = 0.9;
    }

    if(ay > -0.1 && ay > -0.1){
      autumn_cmdvel.linear.y = -0.9;
    }
    else if(ay <= -3 && ay <= -3){
      autumn_cmdvel.linear.y = 0;
    }
  }

  //Winter
  else if(curSeason == 4){
    if(smy > -0.1 && smy > -0.1){
      summer_cmdvel.linear.y = 0;
    }
    else if(smy <= -3 && smy <= -3){
      summer_cmdvel.linear.y = 0.9;
    }
 
    if(spy > -0.1 && spy > -0.1){
      spring_cmdvel.linear.y = 0;
    }
    else if(spy <= -3 && spy <= -3){
      spring_cmdvel.linear.y = 0.9;
    }
   
    if(wy > -0.1 && wy > -0.1){
      winter_cmdvel.linear.y = -0.9;
    }
    else if(wy <= -3 && wy <= -3){
      winter_cmdvel.linear.y = 0;
    }

    if(ay > -0.1 && ay > -0.1){
      autumn_cmdvel.linear.y = 0;
    }
    else if(ay <= -3 && ay <= -3){
      autumn_cmdvel.linear.y = 0.9;
    }
  }
}

/**
  rainStatus function takes is responsible for pulling the rainfall messages in and out of the 
  messageHouse according to the raining boolean set by the changeWeather function.
  author: Oriental Turtles
*/

// Movement of the cloud
void rainStatus(){
  if(raining && (ry > -0.1 && ry > -0.1)){
    rain_cmdvel.linear.y = -0.9;
  }
  else if(raining && (ry <= -3 && ry <= -3)){
    rain_cmdvel.linear.y = 0;
  }
  else if(!raining && (ry <= -3 && ry <= -3)){
    rain_cmdvel.linear.y = 0.9;
  }
  else if(!raining && (ry > -0.1 && ry > -0.1)){
    rain_cmdvel.linear.y = 0;
  }
}

//Call back to listen to odom messages form rain message
void rain_callback(nav_msgs::Odometry msg){
  rx = msg.pose.pose.position.x;
  ry = msg.pose.pose.position.y;
}

//Call back to listen to odom messages form summer message
void summer_callback(nav_msgs::Odometry msg){
  smx = msg.pose.pose.position.x;
  smy = msg.pose.pose.position.y;

}

//Call back to listen to odom messages form winter message
void winter_callback(nav_msgs::Odometry msg){
  wx = msg.pose.pose.position.x;
  wy = msg.pose.pose.position.y;

}

//Call back to listen to odom messages form spring message
void spring_callback(nav_msgs::Odometry msg){
  spx = msg.pose.pose.position.x;
  spy = msg.pose.pose.position.y;

}

//Call back to listen to odom messages form autumn message
void autumn_callback(nav_msgs::Odometry msg){
  ax = msg.pose.pose.position.x;
  ay = msg.pose.pose.position.y;

}


int main(int argc, char **argv){
  dayCounter = 0;
  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument   is the name of the node
  ros::init(argc, argv, "Farm_Control");

  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;

  //advertise() function will tell ROS that you want to publish on a given topic_ to stage
  ros::Publisher farmNode_pub = n.advertise<alpha_two::farmState>("farm_msg", 1000);


  //Creating a publisher that publishes rain messages
  ros::Publisher farmNoderain_pub = n.advertise<alpha_two::rainFall>("rain_msg", 1000);

  //robot_13 - Rain
  //robot_14 - Summer
  //robot_15 - Winter
  //robot_16 - Spring
  //robot_17 - Autumn
  
  //Publishers for weather messages
  ros::Publisher rain_pub = n.advertise<geometry_msgs::Twist>("robot_13/cmd_vel",1000);
  ros::Publisher summer_pub = n.advertise<geometry_msgs::Twist>("robot_14/cmd_vel",1000);
  ros::Publisher winter_pub = n.advertise<geometry_msgs::Twist>("robot_15/cmd_vel",1000);
  ros::Publisher spring_pub = n.advertise<geometry_msgs::Twist>("robot_16/cmd_vel",1000);
  ros::Publisher autumn_pub = n.advertise<geometry_msgs::Twist>("robot_17/cmd_vel",1000);

  //Subscribers  for weather messages
  ros::Subscriber rainOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_13/odom",1000, rain_callback);
  ros::Subscriber summerOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_14/odom",1000, summer_callback);
  ros::Subscriber winterOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_15/odom",1000, winter_callback);
  ros::Subscriber springOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_16/odom",1000, spring_callback);
  ros::Subscriber autumnOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_17/odom",1000, autumn_callback);
  
  ros::Rate loop_rate(10);
  

  sunlight = 80;

  //messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;
  while (ros::ok()){

    changeWeather();
    changeWeatherMessage();
    rainStatus();
    if (raining){
      new_rain_msg.rain = 1;
      farmNoderain_pub.publish(new_rain_msg);
    }
    else{
      new_rain_msg.rain = 0;
      farmNoderain_pub.publish(new_rain_msg);
    }
    
    //Publishing messages to grass and clouds 
	farmNode_pub.publish(new_farm_msg);
	rain_pub.publish(rain_cmdvel);
    
    //Publishing messages to robots responsible for displaying season messages
    summer_pub.publish(summer_cmdvel);
    winter_pub.publish(winter_cmdvel);
    spring_pub.publish(spring_cmdvel);
    autumn_pub.publish(autumn_cmdvel);
	  
	ros::spinOnce();

	loop_rate.sleep();
  }

  return 0;

}

