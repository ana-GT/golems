#include <utility>
#include "ipa_canopen_core/canopen.h"


int main(int argc, char *argv[]) {

    if (argc != 4) {
        std::cout << "Arguments:" << std::endl
                  << "(1) device file" << std::endl
                  << "(2) CAN deviceID (call with 1 for all nodes)" << std::endl
                  << "(3) Baud Rate" << std::endl
                  << "Example: ./homing /dev/pcan32 12 500K" << std::endl;
        return -1;
    }
    double pos_target[] = {0,0,0,0.2,0.2,0.1,0.1,0.1,0.1,0};
    double home_offsets[] = {0,0,0,-0.257,0,-0.132,0.012,-1.404,-0.005,0};
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



	if (CANid > 2 && CANid < 9){

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
	    
	    uint32_t pos = canopen::motor_pos[CANid];
	    double tolerance = 0.0005;
	    bool home = false;
	 
	    while (home == false)
	    {
		if (!canopen::motor_fault[CANid])
		{
			canopen::sendSync();
			std::cout << canopen::mdeg2rad(pos) << std::endl; //-home_offsets[CANid];
		        std::cout << home_offsets[CANid] << std::endl;
			if (canopen::mdeg2rad(pos) <= (pos_target[CANid]+tolerance) && canopen::mdeg2rad(pos) >= (pos_target[CANid]-tolerance))  home = true;
			if (canopen::mdeg2rad(pos) - pos_target[CANid] < 0) negative = false;
			else negative = true;
			
			if (canopen::voltage_enabled[CANid])
			{
				if (home == false){
					if (negative)	pos = pos - 00;
					else 	pos = pos + 00;
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
	    canopen::closeConnection();

	}

	else if (CANid == 1){
	    CANid = 3;
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
						if (negative)	pos = pos - 00;
						else 	pos = pos + 00;
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
		std::cout << "all nodes homed" << std::endl;
	}



}
