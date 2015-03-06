#include <utility>
#include "ipa_canopen_core/canopen.h"


int main(int argc, char *argv[]) {

    if (argc != 5) {
        std::cout << "Arguments:" << std::endl
                  << "(1) device file" << std::endl
                  << "(2) CAN deviceID" << std::endl
                  << "(3) Baud Rate" << std::endl
                  << "Example: ./homing /dev/pcan32 12 500K" << std::endl;
        return -1;
    }
    double home_offsets[] = {0,0,0,-0.257,0,-0.132,0.012,-1.404,-0.005,0};
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
    canopen::sendSync();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    canopen::setOperation(CANid);
    //canopen::sendSDOWrite(CANid, 0x6040, 0, 4, 0x000001F);
    uint32_t pos = canopen::motor_pos[CANid];
    double tolerance = 0.0005;
    bool home = false;
    canopen::sendReleaseBrake(CANid);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    canopen::setBrake(CANid);
    std::cout << "HERE" << std::endl;
}
