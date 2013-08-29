#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "angles/angles.h"
#include "alpha_two/herdingBar.h"
#include <sstream>
#include "math.h"


using namespace std;

//velocity of the robot
double linear_x;
double linear_y;
double angular_z;

//pose of the robot
double px;
double py;
double ptheta;
double theta;

int state;
char herdingBarNumber; // herdingBarNumber refers to the robot number in the world file
char currentField;
char destinationField;

bool isHorizontal = false;

struct instruction_struct{
  int step_count; //how many times do we execute this step before we move to the next step
  double linear_x; //linear velocity in m/s
  double linear_y; //angular velocity
  int next_step; //which step do we move to when we finish this current step
};

void StageOdom_callback(nav_msgs::Odometry msg){
  // This is the call back function to process odometry messages coming from Stage. 	
  px = 5 + msg.pose.pose.position.x;
  py =10 + msg.pose.pose.position.y;
	
  // condition for movement
  // Below code has the following format.
  // herdingBarNumber refers to the robot number in the world file. I.e. first bar is robot_18.
  // (px > XX && state == 0) Checks the upper bound of the herding bar during the herding of sheep.
  // (px < 5 && state == 2) Checks the lower bound of the herding bar, to return it to it's original position.
  if(herdingBarNumber == 18){  
    if((px > 26 && state == 0) || (px < 5 && state == 2)) 
        state = 1;
  }
  else if(herdingBarNumber == 19){
	if((px < -24 && state == 0) || (px > 5 && state == 2)) 
	  state = 1;
  }
  else if(herdingBarNumber == 20){
	if((px < -24 && state == 0) || (px > 5 && state == 2)) 
	  state = 1;
  }
  else if(herdingBarNumber == 21){
	if((px < -18 && state == 0) || (px > 5 && state == 2))
	  state = 1;
  }
  else if(herdingBarNumber == 22){ 
	if((px < -18 && state == 0) || (px > 5 && state == 2)) 
	  state = 1;
  }
  else if(herdingBarNumber == 23){ 
	if((px > 34 && state == 0) || (px < 5 && state == 2)) 
	  state = 1;
  }
  else if(herdingBarNumber == 24){
	if((px < -18 && state == 0) || (px > 5 && state == 2)) 
	  state = 1;
  }
  else if(herdingBarNumber == 25){ 
	if((px > 34 && state == 0) || (px < 5 && state == 2)) 
	  state = 1;
  }	

  //displayed on terminal
  ROS_INFO("Current x position is: %f", px);
  ROS_INFO("Current y position is: %f", py);
}

void StageLaser_callback(sensor_msgs::LaserScan msg){
  //This is the callback function to process laser scan messages
  //you can access the range data from msg.ranges[i]. i = sample number	
}


void addInstruction(vector<instruction_struct>& instruction_vector, int step_count, double linear_x, double linear_y, int next_step){
  instruction_struct *Instruction = new instruction_struct; //create a new instruction_struct
  Instruction->step_count = step_count;
  Instruction->linear_x = linear_x;
  Instruction->linear_y = linear_y;
  Instruction->next_step = next_step;
  instruction_vector.push_back(*Instruction); //add instruction_struct to back of vector
}

int main(int argc, char **argv){

  // HerdingBar message struct
  alpha_two::herdingBar herdingBar_msg;

  //initialize robot parameters
  //Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
  theta = M_PI/2.0;
  px = 26;
  py = 0;
	
  //Initial velocity
  linear_x = 0;
  linear_y = 0;

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
  ros::Publisher HerdingBar_pub = n.advertise<alpha_two::herdingBar>("/herdingBar",1000);

  ros::Rate loop_rate(10);

  //count of how many messages we have sent
  int count = 0;

  //Argument for opening or closing (second argument)
  state = atoi(argv[2]);
  //Argument for herding bar number (first argument)
  herdingBarNumber = atoi(argv[1]);
  
  //currentField = atoi(argv[3]);

  //destinationField = atoi(argv[4]);

  //velocity of this RobotNode
  geometry_msgs::Twist RobotNode_cmdvel;  

  vector <instruction_struct> instruction_vector;

  // Conditions to check gate to move, and call addInstruction
  // Farm1: Vertical herding bar
  if(herdingBarNumber == 18){
    addInstruction(instruction_vector, 7, 0.10, 0.0, 2); // Inward movement
  	addInstruction(instruction_vector, 0, 0, 0.0, 3); // Stop movement
  	addInstruction(instruction_vector, 75, -1, 0.0, 0); // Outward movement
  }
  else if(herdingBarNumber == 19){ // Farm1: Horizontal bar
    addInstruction(instruction_vector, 750, -0.10, 0.0, 1); // Inward movement
  	addInstruction(instruction_vector, 0, 0, 0.0, 2); // Stop movement
  	addInstruction(instruction_vector, 75, 1, 0.0, 0); // Outward movement
  }
  else if(herdingBarNumber == 20){ // Farm1: Horizontal bar
    addInstruction(instruction_vector, 750, -0.10, 0.0, 1); // Inward movement
  	addInstruction(instruction_vector, 0, 0, 0.0, 2); // Stop movement
  	addInstruction(instruction_vector, 75, 1, 0.0, 0); // Outward movement
  }
  else if(herdingBarNumber == 21){ // Farm1: Horizontal bar
    isHorizontal = true; // Check if it needs to be moved across
    addInstruction(instruction_vector, 750, -0.10, 0.0, 1); // Inward movement
    addInstruction(instruction_vector, 75, 0, -0.5, 2);     // Move across
  	addInstruction(instruction_vector, 0, 0, 0.0, 3); // Stop movement
  	addInstruction(instruction_vector, 75, 1, 0.0, 0); // Outward movement
  }
  else if(herdingBarNumber == 22){ // Farm1: Horizontal bar
    isHorizontal = true;                        // Check if it needs to be moved across
    addInstruction(instruction_vector, 750, -0.10, 0.0, 1); // Inward movement
    addInstruction(instruction_vector, 75, 0, -0.5, 2);     // Move across
  	addInstruction(instruction_vector, 0, 0, 0.0, 3); // Stop movement
  	addInstruction(instruction_vector, 75, 1, 0.0, 0); // Outward movement
  }
  else if(herdingBarNumber == 23){ // Farm1: Horizontal bar
    addInstruction(instruction_vector, 750, 0.10, 0.0, 1); // Inward movement
  	addInstruction(instruction_vector, 0, 0, 0.0, 2); // Stop movement
  	addInstruction(instruction_vector, 75, -1, 0.0, 0); // Outward movement
  }
  else if(herdingBarNumber == 24){ // Farm1: Horizontal bar
    addInstruction(instruction_vector, 750, -0.10, 0.0, 1); // Inward movement
  	addInstruction(instruction_vector, 0, 0, 0.0, 2); // Stop movement
  	addInstruction(instruction_vector, 75, 1, 0.0, 0); // Outward movement
  }
  else if(herdingBarNumber == 25){ // Farm1: Horizontal bar
    isHorizontal = true;                        // Check if it needs to be moved across
    addInstruction(instruction_vector, 750, 0.10, 0.0, 1); // Inward movement
    addInstruction(instruction_vector, 75, 0, 0.5, 2);     // Move across
  	addInstruction(instruction_vector, 0, 0, 0.0, 3); // Stop movement
  	addInstruction(instruction_vector, 75, -1, 0.0, 0); // Outward movement
  }

  //keep track of what step we are up to
  int current_step = 0;
  int current_step_count = 0;
  
  while (ros::ok()){
    // Send message once to sheep that herding has begun. 
    ++current_step_count;     
     
    herdingBar_msg.herdingMode = state;
    HerdingBar_pub.publish(herdingBar_msg); 
    
    // Check if bar needs to be moved across
    if(state == 0 && isHorizontal) {
      linear_x = instruction_vector[1].linear_x;
      linear_y = instruction_vector[1].linear_y; 
      current_step++;           
    }
    else if(state == 0){
      linear_x = instruction_vector[0].linear_x;
      linear_y = instruction_vector[0].linear_y;

    }
    else if(state == 1){
      break;
    }
    else if(state == 2){
      linear_x = instruction_vector[2].linear_x;
      linear_y = instruction_vector[2].linear_y;
    }

    RobotNode_cmdvel.linear.x = linear_x;
    RobotNode_cmdvel.linear.y = linear_y;
        
    cout << "Current step: " << current_step_count << "\n";

    //publish the message
    RobotNode_stage_pub.publish(RobotNode_cmdvel);
	
    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    // Check the number of steps moved
    if(current_step > 150) {
      isHorizontal = false;     // No longer needs to be moved across
    }  
    

        
  }//ends while()
	
  RobotNode_cmdvel.linear.x = 0;
  RobotNode_cmdvel.linear.y =0;
        
  cout << "Current step: " << current_step << "\n";
  
  //publish the message
  RobotNode_stage_pub.publish(RobotNode_cmdvel);
  
  return 0;

}//ends main()
