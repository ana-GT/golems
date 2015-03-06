#include <utility>
#include "ipa_canopen_core/canopen.h"
#include <algorithm>
#include <math.h>

bool istrue(bool arr[10])
{
   uint8_t i; 
   uint8_t count;
   for (i=3;i<11;i++) 
   {  
	count += arr[i];
   }
   //std::cout << std::dec << "Count:  " << (uint8_t)count << std::endl;
   if (count == 8)	{std::cout << "Reached"; return true;}
   else return false;
}


int main(int argc, char *argv[]) {

    if (argc != 5) {
        std::cout << "Arguments:" << std::endl
                  << "(1) device file" << std::endl
                  << "(2) CAN deviceID (call with 1 for all nodes)" << std::endl
                  << "(3) Baud Rate" << std::endl
                  << "Example: ./homing /dev/pcan32 12 500K" << std::endl;
        return -1;
    }
    
    double  positions_db[4][10] = {
				{0,	0,	0,	-1.15499,	1.07,	-1.018,		0.126703,	-0.81194,	0.075,		0},
				{0,	0,	0,	0,		1,	2.4,		0,		0.15,		0,		1.5},
				{0,	0,	0,	0,		0,	0,		0,		0,		0,		0},
				{0,	0,	0,	1.570796,		1.570796,	1.570796,		1.570796,		1.570796,		1.570796,		1.570796},
				
					};

    uint8_t position_choose = std::stoi(std::string(argv[4]));
    std::cout << "Here" <<std::endl;
    double *pos_target;
    pos_target = &positions_db[position_choose][0];
double home_offsets[] = {0,0,0,-0.257,0,-0.132,0.012,0.642,-0.005,0};


    double speeds[]= {0,0,0,300,300,300,300,300,300,0};
    //double* pos_target = positions_db[position_choose];


/*if (position_choose == 1)
	{ 
	//this is the test pick up line
    	pos_target = [0,0,0,-1.15499,1.07,-1.018,0.126703,-0.81194,0.075,0];
	}    	
    else if(position_choose == 2)
	{
	//this is fold
    	pos_target = [0,0,0,0,1,2.4,0,0.15,0,1.5];
    	}
    else if(position_choose == 0)
	{
	//this is home
	pos_target = [0,0,0,-0.257,0,-0.132,0.012,-1.404,-0.005,0];
	}*/

	
    uint8_t i;
    for (i=0; i < 10 ; i++)
    {
        pos_target[i] += home_offsets[i];
    }
    std::string deviceFile = std::string(argv[1]);
    uint16_t CANid = std::stoi(std::string(argv[2]));
    canopen::baudRate = std::string(argv[3]);

    bool home_all;
    int home_count = 0;
    if (CANid == 1) home_all = true;

    bool negative; //direction bool 

    if (CANid == 1){
	    CANid = 3;
	    bool reached[10] = {1,1,1, 0,0,0, 0,0,0 ,0};

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
	    for (i=3;i<9;i++)	canopen::setOperation(i);
	    //canopen::sendSDOWrite(CANid, 0x6040, 0, 4, 0x000001F);
	    double tolerance = 0.015;
  	    canopen::sendSync();
	    double pos_diffs[10];
	    for (i=3;i<9;i++) 	pos_diffs[i] = abs(*(pos_target+i)-canopen::mdeg2rad(canopen::motor_pos[i]));
	    double max_distance = 2.5*3.1415;
	    uint8_t max_velocity = 400;
	
	   
	    while (!istrue(reached))
	    {
		    for (CANid=3; CANid<9;CANid++)
		    {
			uint32_t pos = canopen::motor_pos[CANid];
			//if (CANid == 7) std::cout << canopen::mdeg2rad(pos) << std::endl;
			if (!canopen::motor_fault[CANid])
				{
					canopen::sendSync();
					//std::cout << canopen::mdeg2rad(pos) << std::endl; //-home_offsets[CANid];
					//std::cout << *(pos_target+CANid) << std::endl;
					if ((canopen::mdeg2rad(pos) <= *(pos_target+CANid)+tolerance) && ( canopen::mdeg2rad(pos) >= *(pos_target+CANid) - tolerance))  					
					{
						reached[CANid] = true;
					}
					if (canopen::mdeg2rad(pos) - *(pos_target+CANid) < 0) negative = false;
					else negative = true;
					
  					//speeds[CANid] = abs(canopen::mdeg2rad(pos) - *(pos_target+CANid))/pos_diffs[CANid]*speeds[CANid]  + 20;
					//speeds[CANid] = pos_diffs[CANid]/max_distance[CANid]
				
					if (canopen::voltage_enabled[CANid])
					{
						if (reached[CANid] == false){
							
							if (negative)	pos = pos - (canopen::rad2mdeg(0.008)); // - speeds[CANid];
							else 	pos = pos + (canopen::rad2mdeg(0.008)); //speeds[CANid];
						}
						else  pos = canopen::motor_pos[CANid];
					}
					else pos = canopen::motor_pos[CANid];
					canopen::sendPDO(CANid, pos, true);
					//std::this_thread::sleep_for(std::chrono::milliseconds(3));
			    		
				}
				else canopen::setOperation(CANid);
		    }
		
	    }
	    std::cout << "all nodes homed" << std::endl;

	    /*
	    while (CANid <= 8){
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
		    canopen::setOperation(CANid);
		    //canopen::sendSDOWrite(CANid, 0x6040, 0, 4, 0x000001F);
		    uint32_t pos = canopen::motor_pos[CANid];
		    double tolerance = 0.0005;
		    bool home = false;
		 
		    while (home == false)
		    {
			if (!canopen::motor_fault[CANid])
			{
				canopen::sendSync();
				std::cout << canopen::mdeg2rad(pos) << std::endl; //-home_offsets[CANid];
				std::cout << pos_target[CANid] << std::endl;
				if (canopen::mdeg2rad(pos) <= (pos_target[CANid]+tolerance) && canopen::mdeg2rad(pos) >= (pos_target[CANid]-tolerance))  home = true;
				if (canopen::mdeg2rad(pos) - pos_target[CANid] < 0) negative = false;
				else negative = true;
				
				if (canopen::voltage_enabled[CANid])
				{
					if (home == false){
						if (negative)	pos = pos - 60;
						else 	pos = pos + 60;
					}
					else  pos = canopen::motor_pos[CANid];
				}
				else pos = canopen::motor_pos[CANid];
				canopen::sendPDO(CANid, pos, true);
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
		    		
			}
			else canopen::setOperation(CANid);
		    }
		    std::this_thread::sleep_for(std::chrono::seconds(1));
		
		    std::cout << "node homed" << std::endl;
		    //canopen::sendSDOWrite(CANid, 0x6040, 0, 4, 0x000000F);
		    canopen::sendSDOWrite(CANid, 0x6040, 0, 4, 0x0000006);
		    CANid = CANid +1;
		    canopen::closeConnection();
		}
		*/
		
		
	}



}
