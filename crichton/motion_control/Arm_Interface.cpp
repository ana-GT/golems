/**
 * @file Arm_Interface.cpp
 */

#include "Arm_Interface.h"


Arm_Interface::Arm_Interface() {
  mMaxDev = 0.1;
}

Arm_Interface::~Arm_Interface() {
  
}


/**
 * @function followTrajectory
 */
bool Arm_Interface::followTrajectory( const std::list<Eigen::VectorXd> &_path ) {
  
  // Build a message
  struct sns_msg_path_dense* msg = sns_msg_path_dense_alloc( _path.size(),
							     (*(_path.begin())).size() );

  msg->n_dof = (*(_path.begin())).size();
  msg->n_steps = _path.size();
  
  sns_msg_header_fill( &msg->header );
  msg->header.n = msg->n_dof;

  // Fill data in message
  int counter = 0;
  std::list<Eigen::VectorXd>::const_iterator it;
  for( it = _path.begin(); it != _path.end(); ++it ) {
    for( int j = 0; j < (*it).size(); ++j ) {
      msg->x[counter] = (*it)(j);
      counter++;
    }
  }

  // Set message duration
  double tsec = 0.05;
  int64_t dt_nsec = tsec*1e9;

  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "Clock_gettime failed: %s \n", strerror(errno) );   
  }
  sns_msg_set_time( &msg->header, &mNow, dt_nsec );

  // Send message
  ach_status_t r;
  r = ach_put( mChan_output, msg, sns_msg_path_dense_size(msg) );

  if( r!= ACH_OK ) { printf("\t * [ERROR] Error sending arm path \n"); }
  else { printf("\t * [INFO] Arm path was sent all right\n"); }

  return true;  
}
