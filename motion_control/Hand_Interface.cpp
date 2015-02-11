/**
 * @file Hand_Interface.cpp
 */
#include "Hand_Interface.h"

/**
 * @function Hand_Interface 
 */
Hand_Interface::Hand_Interface() {

}

/**
 * @function ~Hand_Interface 
 */
Hand_Interface::~Hand_Interface() {

}

/**
 * @function goToConfiguration 
 */
bool Hand_Interface::goToConfiguration( const Eigen::VectorXd &_config,
					double _dt ) {

  return control_n( _config.size(),
		    _config,
		    _dt,
		    mChan_output,
		    SNS_MOTOR_MODE_POS );  
}
