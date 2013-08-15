#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"

#include <sstream>
#include "math.h"

using namespace std;

// Global variables and objects
ros::Publisher RobotNode_stage_pub;
geometry_msgs::Twist RobotNode_cmdvel;

bool showDebug = false;

bool makeMovement = true;

#define sampleNumber 10 // represents number of samples in world file.


void StageOdom_callback(nav_msgs::Odometry msg)
{

}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
    // TODO rotate to avoid object until a majority of the beams are free, implying that path way forward is open for traversal.
    // TODO Currently the robot only turns clockwise, later on 
    // the sheep should rotate depending on beam density.
    // TODO find a way to detect type of object that is in view or has collided with.
	
    if(showDebug)ROS_INFO("------------------------ Intensity ----------------------------");
    for(unsigned int i = 0; i < msg.intensities.size(); ++i){
    	// Either 1 or 0 is returned. 1 if an object is view 0 otherwise.
    	double curRange = msg.intensities[i];   
    	if(showDebug)ROS_INFO("Current intensity is    : %f", curRange);
    }

    if(showDebug)ROS_INFO("-------------------------- Range -----------------------------");
    //publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
    
    
    int curLowestIndex = 0;
    
    for(unsigned int j = 1; j < msg.ranges.size(); ++j){
      
        if(msg.ranges[curLowestIndex] > msg.ranges[j]){
              //currentLowest = msg.ranges[j];
              curLowestIndex = j;
        }
      
      
        double curRange = msg.ranges[j];
        
        if(showDebug)ROS_INFO("index: %d", curLowestIndex);

   
        // When a beam is 1unit in length the robot stops moving and
        // and begins rotating to avoid the obstacle.
        if(msg.ranges[curLowestIndex] < 1.5 && msg.ranges[curLowestIndex] > 0.8){
            if(showDebug)ROS_INFO("Collision at beam: %d Range: %f",j,curRange);
            makeMovement = false;
            
	        // Robot moves forward at a slow pace of 0.1m/s
            RobotNode_cmdvel.linear.x = 0.1;
            
            if(curLowestIndex < sampleNumber/2){
                RobotNode_cmdvel.angular.z = (M_PI / 18) * 5;
            }
            else{
                RobotNode_cmdvel.angular.z = -(M_PI / 18) * 5;
            }
            
            RobotNode_stage_pub.publish(RobotNode_cmdvel);
            
      }
      else if(msg.ranges[curLowestIndex] <= 0.8){
         // Once no potential collisions can be detected robot(sheep) continues to move forward.
            RobotNode_cmdvel.linear.x = 0;
            RobotNode_cmdvel.angular.z = (M_PI / 18) * 5;
            if(showDebug)ROS_INFO("Current range is    : %f", curRange);
      }
      else{
        // Once no potential collisions can be detected robot(sheep) continues to move forward.
            RobotNode_cmdvel.linear.x = 1;
            RobotNode_cmdvel.angular.z = 0;
           if(showDebug) ROS_INFO("Current range is    : %f", curRange);
      }
    }	
    if(showDebug)ROS_INFO("-------------------------- END -----------------------------");
} //ends StageLaser_callback()



int main(int argc, char** argv){

    std::stringstream rName;
    rName.str("");
    rName << "RobotNode" << argv[1];	
    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
    ros::init(argc, argv, rName.str());
    ros::NodeHandle n;

  
    rName.str("");
    rName << "robot_" << argv[1]<<"/cmd_vel";
    RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>(rName.str(),1000); 

    rName.str("");
    rName << "robot_" << argv[1]<<"/base_scan";
    ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>(rName.str(),1000,StageLaser_callback);

    rName.str("");
    rName << "robot_" << argv[1]<<"/odom";
    ros::Subscriber StageOdom_sub = n.subscribe<nav_msgs::Odometry>(rName.str(),1000,StageOdom_callback);

    int count = 0;
    ros::Rate r(10); // 10 cycles per second

    while(n.ok()){
        ++count;
        ros::spinOnce();
        r.sleep();
    }//ends while()

}//ends main()
