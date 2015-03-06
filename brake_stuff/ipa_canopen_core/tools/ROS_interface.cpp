#include <utility>
#include "ipa_canopen_core/canopen.h"
#include <algorithm>
#include <math.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <iterator> 
#include <vector>
#include <stdlib.h>
#include <stdio.h>

//import ArmJointAngles message 
#include <segway_interface/ArmJointAngles.h>

#include <segway_interface/ArmCurrentPos.h>
#include <segway_interface/ArmState.h>




double pos_out[10];
double pos_raw[] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0};

double home_offsets[] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0}; //for fine tuning

void getCurrentPos();

bool finished_move = false;

bool goalReached = false;
bool alreadySentGS0 = false;

bool move(double pos_target[]);

double pos_act[] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0};

int get_max_pos(double * array, int size)
{
    double max=array[0];
    int max_pos=0;

    int i;
    for (i=1; i<size; i++)
    {
        if (max<array[i])
        {
            max=array[i];
            max_pos=i;
        }
    }

    return max_pos;
}

bool istrue(bool arr[10])
{
   uint8_t i; 
   uint8_t count = 0;
   for (i=3;i<9;i++) 
   {  
	count +=arr[i];

   }
   //std::cout << std::dec << "Count:  " << (uint8_t)count << std::endl;
   if (count >= 6)	
	{
	//std::cout << "Reached"; 
	return true;
	}
   else {
	return false;
	}
}

void ros_arm_GS_pub(ros::Publisher pub){

segway_interface::ArmState msg;
while (ros::ok()){
if (goalReached == true){ 
	msg.GS = 1;
	goalReached = false;
	pub.publish(msg);
	}
else if (alreadySentGS0 == false){
	msg.GS = 0;
	pub.publish(msg);
	alreadySentGS0 = true;
}
}

}



void ros_arm_pub(ros::Publisher pub)
{
segway_interface::ArmCurrentPos array;
 while (ros::ok()){
 
 array.data.clear();

  //pos_act[3] = pos_act[3] *-1; //account for 2nd node on schunk inverted

    	for (int i = 2; i < 9; i++)
	{
	//assign array 
		array.data.push_back(pos_act[i]);
	}

    	pub.publish(array);
	
	
  }
}

void arrayCallback(const segway_interface::ArmJointAngles::ConstPtr& msg)
{

std::vector<float> p =  msg->joint_angles ;

//display the vector
/*
copy(p.begin(), p.end(), std::ostream_iterator<float>(std::cout, " "));
    std::cout << std::endl; 
*/

for (int index=0; index<6; ++index) {
	       // std::cout << p[index] <<std::endl;
		pos_raw[index+3] = p[index];
		if (index == 2){
			pos_raw[index+3] = pos_raw[index+3] *-1; //account for 2nd node on schunk inverted
			
		}
		if (index == 5){
			pos_raw[index+3] = pos_raw[index+3] *-1; //account for 2nd node on schunk inverted
			
		}
	    }

	move(pos_raw);
}


void rosSpin(){
	while (ros::ok()) ros::spinOnce();
	sleep(0.1);

}

int main(int argc, char *argv[]) {
 /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "arm_listener");
  
 
	

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  
 ros::NodeHandle n;


  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("arm_joints_target", 1000, arrayCallback);
  ros::Publisher pub = n.advertise<segway_interface::ArmCurrentPos>("arm_current_pos", 1000);
  ros::Publisher pub_GS = n.advertise<segway_interface::ArmState>("ArmState/GS_ROSinterface", 1000);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

std::thread pos_pub(ros_arm_pub, pub);
std::thread rosSpinner(rosSpin);
std::thread GS_pub(ros_arm_GS_pub, pub_GS);
GS_pub.join();
pos_pub.join();
rosSpinner.join();



   return 0;

}

void getCurrentPos(){

	std::string deviceFile = "/dev/pcan32";
	uint16_t CANid = 3;
	canopen::baudRate = "500K";
	   if (!canopen::openConnection(deviceFile,canopen::baudRate)){
		std::cout << "Cannot open CAN device; aborting." << std::endl;
		exit(EXIT_FAILURE);
	    }
	    else{
		std::cout << "Connection to CAN bus established" << std::endl;
		
	    }

	    canopen::devices[ CANid ] = canopen::Device(CANid);
	    std::this_thread::sleep_for(std::chrono::milliseconds(10));
	
	    canopen::initListenerThread(canopen::defaultListener);
	
	    canopen::sendSync();
	    std::this_thread::sleep_for(std::chrono::milliseconds(100));
            canopen::init(1);
	    canopen::sendSync();
	    uint8_t i;
	    for (i=3;i<9;i++)	canopen::setOperation(i);
	
	    for (i=3;i<9;i++){ 	
		pos_out[i] = canopen::mdeg2rad(canopen::motor_pos[i]);
		}
	

}


bool move(double pos_target[]){


/*
    std::fstream infile ("/home/seg/catkin_ws/src/ipa_canopen/ipa_canopen_core/tools/config/calib.txt");

    std::string sLine;

    infile >> sLine;
    std::string pos_7_old = sLine.data();
    std::cout << "current position of 7: " <<pos_7_old;
    infile.close();
    
    */
 
   


    double speeds[]= {0,0,0,300,300,300,300,300,300,0};
   

	
    
    std::string deviceFile = "/dev/pcan32";
    uint16_t CANid = 1;
    canopen::baudRate = "500K";

    bool home_all;
    int home_count = 0;
    if (CANid == 1) home_all = true;

    bool negative; //direction bool 

    if (CANid == 1){
	    
	    CANid = 3;
	    bool reached[10] = {1,1,1, 0,0,0, 0,0,0 ,1};
	
	   if (!canopen::openConnection(deviceFile,canopen::baudRate)){
		std::cout << "Cannot open CAN device; aborting." << std::endl;
		exit(EXIT_FAILURE);
	    }
	    else{
		std::cout << "Connection to CAN bus established" << std::endl;
		
	    }
	

       	    
	    canopen::devices[ CANid ] = canopen::Device(CANid);
	    std::this_thread::sleep_for(std::chrono::milliseconds(10));
	
	    canopen::initListenerThread(canopen::defaultListener);
	
	    canopen::sendSync();
	    std::this_thread::sleep_for(std::chrono::milliseconds(100));
            canopen::init(1);
	    canopen::sendSync();
	    uint8_t i;
	    for (i=3;i<9;i++)	canopen::setOperation(i);

	    
    	    for (i=0; i < 10 ; i++)
    		{
        	pos_target[i] += home_offsets[i];
		//std::cout << pos_target[i];
    	     }
	    
	    double tolerance = 0.015;
  	    
	    double pos_diffs[10];
	    for (i = 3; i <9; i++){
		pos_diffs[i] = fabs(pos_target[i] - (double)canopen::mdeg2rad(canopen::motor_pos[i]));
		std::cout << pos_diffs[i] << std::endl;
		}

	    int max_distance_index = get_max_pos(pos_diffs, 10);
	    double max_distance = pos_diffs[max_distance_index];
	    std::cout << "max distance is: " << max_distance << std::endl;
	 
	    double max_velocity = 0.02;
	   
	    double speeds[10];
	    for (i=3; i<9; i++) {
		speeds[i] = max_velocity * pos_diffs[i] / max_distance;
		std::cout << speeds[i] << std::endl;
		}
	    

	    while (istrue(reached) == false)
	    {
		    for (CANid=3; CANid<9;CANid++)
		    {
			uint32_t pos = canopen::motor_pos[CANid];
			pos_act[CANid] = canopen::mdeg2rad(pos) + home_offsets[CANid];
		
			if (!canopen::motor_fault[CANid])
				{
					canopen::sendSync();
					
					if ((canopen::mdeg2rad(pos) <= (pos_target[CANid]+tolerance)) && (canopen::mdeg2rad(pos) >= (pos_target[CANid] - tolerance)) ) 					
					{
						reached[CANid] = 1;
					}
					if (canopen::mdeg2rad(pos) - pos_target[CANid] < 0) negative = false;
					else negative = true;
					
  					
					if (canopen::voltage_enabled[CANid])
					{
						if (reached[CANid] == false){
							if (negative)	pos = pos - (canopen::rad2mdeg(speeds[CANid])); 
							else 	pos = pos + (canopen::rad2mdeg(speeds[CANid])); 
							
							
						}
						else  pos = canopen::motor_pos[CANid];
					}
					else pos = canopen::motor_pos[CANid];
					canopen::sendPDO(CANid, pos, true);
					
				}
				else canopen::setOperation(CANid);
		    }
		
	    }
	    std::cout << "all nodes here" << std::endl;
	    goalReached = true;
      	    sleep(0.5);
	    return true;
	    
		
	}



}
