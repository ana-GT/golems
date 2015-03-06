#include <utility>
#include "ipa_canopen_core/canopen.h"


int main(int argc, char *argv[]) {

    if (argc != 5) {
        std::cout << "Arguments:" << std::endl
                  << "(1) device file" << std::endl
                  << "(2) CAN deviceID" << std::endl
                  << "(3) Baud Rate" << std::endl
	          << "(4) direction bool " << std::endl
                  << "Example: ./homing /dev/pcan32 12 500K" << std::endl;
        return -1;
    }
    
    std::string deviceFile = std::string(argv[1]);
    uint16_t CANid = std::stoi(std::string(argv[2]));
    canopen::baudRate = std::string(argv[3]);
    bool negative = std::stoi(std::string(argv[4]));
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
    canopen::init(1);
    canopen::sendSync();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    canopen::setOperation(CANid);
    
    uint32_t pos = canopen::motor_pos[CANid];
    double tolerance = 0.0005;
    bool home = false;
    while (1)
    {
        if (!canopen::motor_fault[CANid])
        {
		canopen::sendSync();
		std::cout << canopen::mdeg2rad(pos) << std::endl; 
                
        	if (canopen::voltage_enabled[CANid])
        	{
			if (home == false){
				if (negative)	pos = pos - 30;
				else 	pos = pos + 30;
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
    
   
}
