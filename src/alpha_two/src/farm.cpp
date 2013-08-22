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
int dayCounter;
double px,py;
bool raining = false;
// Changes the weather conditions of the farm
void changeWeather(){
  
  dayCounter = dayCounter%732;
  dayCounter++;
  //Rainfall is determined randomly according to the current season.
  //Spring
  if(dayCounter < 100){

    new_farm_msg.rainfall = rand()%5;

  //Winter
  }else if (dayCounter < 460){
    new_farm_msg.rainfall = rand()%6;

  //Summer
  }else if (dayCounter < 532){        
    new_farm_msg.rainfall = rand(); 
  //Autumn
  }else if (dayCounter < 732){        
    new_farm_msg.rainfall = rand()%3;
  }else{
    dayCounter = 0;
  }
  printf("day counter: %d\n",dayCounter);
  if(dayCounter>0 && dayCounter<300){
    raining = true;
  } else if (dayCounter >= 300 && dayCounter < 600){
    raining = false;
  } else{
    raining = true;
  }
  
  //Weather changes applied to each field.  
  new_farm_msg.f1_soil_condition = abs((new_farm_msg.f1_soil_condition +new_farm_msg.rainfall)%100);
  new_farm_msg.f2_soil_condition = abs((new_farm_msg.f2_soil_condition +new_farm_msg.rainfall)%100);
  new_farm_msg.f3_soil_condition = abs((new_farm_msg.f3_soil_condition +new_farm_msg.rainfall)%100);
  new_farm_msg.f4_soil_condition = abs((new_farm_msg.f4_soil_condition +new_farm_msg.rainfall)%100);
  
  
  //new_farm_msg.f4_soil_condition += int(float(new_farm_msg.f4_soil_condition)*(float(new_farm_msg.rainfall)/100.0)) -5.0;
  
}

int main(int argc, char **argv)
{
	dayCounter = 0;
  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument   is the name of the node
  ros::init(argc, argv, "Farm_Control");

  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;

  //advertise() function will tell ROS that you want to publish on a given topic_
  //to stage

  ros::Publisher farmNode_pub = n.advertise<alpha_two::farmState>("farm_msg", 1000);

  ros::Publisher farmNoderain_pub = n.advertise<alpha_two::rainFall>("rain_msg", 1000);
  
  
  ros::Rate loop_rate(10);
  
  new_farm_msg.rainfall = 0;
  new_farm_msg.f1_soil_condition = 100;
   
  new_farm_msg.f2_soil_condition = 50;
  
  new_farm_msg.f3_soil_condition = 70;
   
  new_farm_msg.f4_soil_condition = 85;
  ////messages
  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;
  while (ros::ok())
  {

    changeWeather();
    if (raining){
      new_rain_msg.rain = 1;
      farmNoderain_pub.publish(new_rain_msg);
    }else{
      new_rain_msg.rain = 0;
      farmNoderain_pub.publish(new_rain_msg);
    }
     
	  farmNode_pub.publish(new_farm_msg);
	  
	  
	  ros::spinOnce();

	  loop_rate.sleep();
  }

return 0;

}

