/**
 * @file Arm_Interface.h
 * @brief Instance class to interface with arm (7 joints for Crichton)
 */
#pragma once

#include "motion_control/Control_Interface.h"

/**
 * @class Arm_Interface
 */
class Arm_Interface : public Control_Interface {

 public:
  Arm_Interface();
  ~Arm_Interface();

  bool followTrajectory( const std::list<Eigen::VectorXd> &_path );

 protected:
  double mMaxDev; // Max. Deviation in path following
};
