/***
 * @file base_control.h
 */
#pragma once

#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <Eigen/Dense>
#include <list>


/**
 * @class BaseControl
 */
class BaseControl {
  
 public:
  BaseControl();
  ~BaseControl();

  void set_numJoints( int _N );
  void set_channels( ach_channel_t* _ref_chan,
		     ach_channel_t* _state_chan );
  bool update();
  void get_state( Eigen::VectorXd &_q,
		  Eigen::VectorXd &_dq );
  bool followTrajectory( const std::list<Eigen::VectorXd> &_path,
			 const Eigen::VectorXd &_maxAccel,
			 const Eigen::VectorXd &_maxVel );
  
  
  /** GENERAL FUNCTIONS */
  bool update_n( size_t n,
		 Eigen::VectorXd &_q,
		 Eigen::VectorXd &_dq,
		 ach_channel_t *chan,
		 struct timespec *ts );

  bool control_n( size_t n,
		  const Eigen::VectorXd &_x,
		  double tsec,
		  ach_channel_t* chan,
		  int mode = SNS_MOTOR_MODE_VEL );

  bool reset_n( size_t n,
		double tsec,
		ach_channel_t* chan );

 protected:
  int mN;
  struct timespec mNow;
  double mFreq;
  double mDt;
  double mDq_thresh;
	int64_t mVALID_NS;

  ach_channel_t* mChan_ref;
  ach_channel_t* mChan_state;

  Eigen::VectorXd mq;
  Eigen::VectorXd mdq;


  double mMaxDev; // Max. deviation in path following
  
};
