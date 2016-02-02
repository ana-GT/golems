/**
 * @file base_dual_control.h
 * @brief Control left arm, right arm or both at the same time
 */

#pragma once

#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <Eigen/Dense>
#include "motion_control/base_control.h"

/**
 * @class BaseDualControl
 */
class BaseDualControl {

 public:
  BaseDualControl();
  ~BaseDualControl();

  void set_numJoints( int _N );
  void set_channels( int _i,
		     ach_channel_t* _ref_chan,
		     ach_channel_t* _state_chan );
  bool update();
  bool followTrajectory( int _side,
			 const std::list<Eigen::VectorXd> &_path,
			 const Eigen::VectorXd &_maxAccel,
			 const Eigen::VectorXd &_maxVel );

  bool followDualTrajectory( const std::list<Eigen::VectorXd> &_pathLeft,
			     const std::list<Eigen::VectorXd> &_pathRight,
			     const Eigen::VectorXd &_maxAccel,
			     const Eigen::VectorXd &_maxVel );
    
  
  
 private:
  int mN;
  struct timespec mNow;
  double mFreq;
  double mDt;
  double mDq_thresh;
  int64_t mVALID_NS;

  double mMaxDev; // Max. deviation in path following
  
  BaseControl mBc[2];
  Eigen::VectorXd mq[2];
  Eigen::VectorXd mdq[2];

};
