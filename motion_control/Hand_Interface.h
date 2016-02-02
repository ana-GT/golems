/**
 * @file Hand_Interface.h
 * @brief Instance class to interface with hand (7 joints for Crichton)
 */
#pragma once

#include "motion_control/Control_Interface.h"

/**
 * @class Hand_Interface
 */
class Hand_Interface : public Control_Interface {

 public:
  Hand_Interface();
  ~Hand_Interface();

  bool goToConfiguration( const Eigen::VectorXd &_config,
			  double _dt );
  
 protected:

};


