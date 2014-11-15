/**
 * @file basicControl.h 
 */

#pragma once

#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <Eigen/Dense>

#define ARM_AXES 7
#define HAND_AXES 7

/**
 * @class BasicControl
 */
class BasicControl {

 public:
  
  BasicControl();
  ~BasicControl();

  /**< Helpers */
  std::vector<Eigen::VectorXd>  preprocessPath( const std::vector<Eigen::VectorXd> _path  );

  bool update_n( size_t n,
		 double *q,
		 double *dq,
		 ach_channel_t *chan,
		 struct timespec *ts );
  bool control_n( size_t n, 
		  double *x,
		  double tsec,
		  ach_channel_t* chan,
		  int mode = SNS_MOTOR_MODE_POS );
  
  bool reset_n( size_t n, double tsec, ach_channel_t* chan );



 protected:

  struct timespec now;
  
  double mFreq;
  double mDt;
  double mDq_thresh;


};
