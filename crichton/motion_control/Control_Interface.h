/**
 * @file Control_Interface.h
 * @brief Base class for interfacing to can402 motor commands
 */
#pragma once

#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <Eigen/Dense>
#include <list>

/**
 * @class Control_Interface
 */
class Control_Interface {

 public:
  Control_Interface();
  ~Control_Interface();

  void set_numJoints( int _N );
  int get_numJoints() const { return mN; }
  void set_channels( ach_channel_t* _chan_state,
		     ach_channel_t* _chan_output );

  bool update();
  void get_state( Eigen::VectorXd &_q,
		  Eigen::VectorXd &_dq );

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
  
 protected:

  int mN;

  ach_channel_t* mChan_state;
  ach_channel_t* mChan_output;

  Eigen::VectorXd mq;
  Eigen::VectorXd mdq;

  struct timespec mNow;
  double mVALID_NS;
  
};

