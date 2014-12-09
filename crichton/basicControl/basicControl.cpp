/**
 * @function basicControl.cpp
 */
#include "basicControl.h"


/**
 * @function basicControl
 */
BasicControl::BasicControl() {

  mFreq = 100.0; // Hz
  mDt = 1.0 / mFreq;
  mDq_thresh = 0.1;
}

/**
 * @function ~BasicControl
 */
BasicControl::~BasicControl() {

}

/**
 * @function preprocessPath
 */ 
std::vector<Eigen::VectorXd> BasicControl::preprocessPath( const std::vector<Eigen::VectorXd> _path  ) {

  std::vector<Eigen::VectorXd> tightPath;
  Eigen::VectorXd p_prev, p_next, p_add, p_step, dp;
  int numSteps;
  double dang = 0.05; // 2.5 degrees
  double dangN = dang*sqrt( _path[0].size() );
  
  printf("dangN: %f dang: %f \n", dangN, dang );

  // Add first point
  tightPath.push_back( _path[0] );

  // Evaluate each consecutive point before starting
  for( size_t i = 0; i < _path.size() - 1; ++i ) {
    p_prev = _path[i];
    p_next = _path[i+1];
    dp = (p_next - p_prev);
 
    // If the distance between consecutive points is too big, reduce it in smaller steps
    numSteps = ceil( dp.norm() / (double)dangN ); 
    printf("[%d] norm: %f Num steps: %d \n", i, dp.norm(), numSteps );

    if( numSteps >= 1 ) {

      p_step = ( dp / dp.norm() )*dangN;
      for( size_t j = 1; j <= numSteps; ++j ) {

	p_add = p_prev + j*p_step;
	if( j == numSteps ) { p_add = p_next; } 

	tightPath.push_back( p_add );
      } // end for

    } 
    // Else, just add the new point
    else {
      tightPath.push_back( p_next );
    }
      } // end for

  return tightPath;
}


/**
 * @function update_n
 * @brief Read motor state from channel chan and store it in q and dq variables
 */ 
bool BasicControl::update_n( size_t n,
				 double *q,
				 double *dq,
				 ach_channel_t *chan,
				 struct timespec *ts ) {
  
  size_t frame_size;
  void *buf = NULL;
  ach_status_t r = sns_msg_local_get( chan, &buf,
				      &frame_size,
				      ts, 
				      ACH_O_LAST | (ts ? ACH_O_WAIT : 0 ) );
  switch(r) {
    
  case ACH_OK:
  case ACH_MISSED_FRAME: {
    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state*)buf;
    if( n == msg->header.n &&
	frame_size == sns_msg_motor_state_size_n((uint32_t)n) ) {
      for( size_t j = 0; j < n; ++j ) {
	q[j] = msg->X[j].pos;
	dq[j] = msg->X[j].vel;
      }
      return true;
    } else {
      SNS_LOG( LOG_ERR, "Invalid motor_state message \n" );
      return false;
    }
  } break;
    
  case ACH_TIMEOUT:
  case ACH_STALE_FRAMES:
  case ACH_CANCELED:
    break;
  default:
    SNS_LOG( LOG_ERR, "Failed ach_get: %s \n", ach_result_to_string(r) );
  } // end switch

  return false;

}

/**
 * @function control_n
 * @brief Sends a motor ref message of size n and data x, to be executed in tsec seconds. 
 */
bool BasicControl::control_n( size_t n, 
			      double *x,
			      double tsec,
			      ach_channel_t* chan,
			      int mode ) {
  
  // Create the message
  struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc(n);
  sns_msg_header_fill( &msg->header );

  switch( mode ) {
  case SNS_MOTOR_MODE_POS:
    msg->mode = SNS_MOTOR_MODE_POS;
    break;
  case SNS_MOTOR_MODE_VEL:
    msg->mode = SNS_MOTOR_MODE_VEL;
    break;
  default:
    return false;
  }

  msg->header.n = n;
  AA_MEM_CPY( msg->u, x, n );


  // Duration from now + tnano
  int64_t tnano = (int64_t)(tsec*1e9);
  if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ){
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }
  sns_msg_set_time( &msg->header, &now, tnano );

  // Send
  ach_status_t r;
  r = ach_put( chan, msg, sns_msg_motor_ref_size(msg) );

  if( r != ACH_OK ) { return false; } 
  else { return true; }
}

/**
 * @function reset
 */
bool BasicControl::reset_n( size_t n, 
			    double tsec, 
			    ach_channel_t* chan ) {
  
  // Create the message
  struct sns_msg_motor_ref* msg = sns_msg_motor_ref_local_alloc(n);
  sns_msg_header_fill( &msg->header );
  msg->mode = SNS_MOTOR_MODE_RESET;
  msg->header.n = n;
  
  // Duration from now + tnano
  double tnano = tsec*1e9;
  struct timespec now;
  if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ){
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );
  }
  sns_msg_set_time( &msg->header, &now, tnano );
  
  // Send
  ach_status_t r;
  r = ach_put( chan, msg, sns_msg_motor_ref_size(msg) );
  
  if( r == ACH_OK ) {
    return true;
  } else {
    return false;
  }
}
