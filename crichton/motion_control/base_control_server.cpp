/**
 * @file base_control_server.cpp
 */
#include <sns.h>
#include <unistd.h>
#include <ach.h>
#include "base_control.h"

#include <list>

ach_channel_t ref_chan;
ach_channel_t state_chan;
ach_channel_t traj_chan;

BaseControl mBc;
double mFreq = 100.0;
double mDt = 1.0 / mFreq;
double mMaxVel = 0.18;
double mMaxAccel = 0.1;

struct timespec t_now;
struct timespec t_timeout;

int num_joints = 2;

/** Functions */
bool poll_traj_chan();



/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open channels
  sns_chan_open( &state_chan, "state-left", NULL );
  sns_chan_open( &ref_chan, "ref-left", NULL );
  sns_chan_open( &traj_chan, "traj-left", NULL );

  mBc.set_numJoints( num_joints );
  mBc.set_channels( &ref_chan, &state_chan );

  // Loop and wait tills a trajectory is sent
  while( true ) {
    poll_traj_chan();
    usleep(1e6*mDt);    
  }

  return 0;
}

/**
 * @function poll_traj_chan
 * @brief Checks if a message is received through traj_chan
 */
bool poll_traj_chan() {

  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_now ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return 0;
  }

  t_timeout = sns_time_add_ns( t_now, 1000*1000*1 );
  
  std::list<Eigen::VectorXd> path;
  size_t frame_size;
  void* buf = NULL;
  ach_status_t r = sns_msg_local_get( &traj_chan,
				      &buf,
				      &frame_size,
				      &t_timeout,
				      ACH_O_LAST | ACH_O_WAIT );

  switch(r) {

  case ACH_OK:
  case ACH_MISSED_FRAME: {
    struct sns_msg_path_dense* msg = (struct sns_msg_path_dense*) buf;
    if( frame_size == sns_msg_path_dense_size(msg) ) {

      // Store trajectory
      int n_dofs, n_steps;
      n_dofs = msg->n_dof;
      n_steps = msg->n_steps;

      if( n_dofs != num_joints ) {
	printf( "\t * [ERROR] Received a trajectory but the n_dofs is not expected \n" );
      }
      
      printf("\t * [INFO] Received trajectory with %d points and %d dofs \n", n_steps, n_dofs );

      Eigen::VectorXd maxVel, maxAccel;
      maxVel = mMaxVel*Eigen::VectorXd::Ones( n_dofs );
      maxAccel = mMaxAccel*Eigen::VectorXd::Ones( n_dofs );

      int counter = 0;
      for( int i = 0; i < n_steps; ++i ) {
	Eigen::VectorXd p(n_dofs);
	for( int j = 0; j < n_dofs; ++j ) {
	  p(j) = msg->x[counter];
	  counter++;
	}
	std::cout << "P["<<i<<"]: "<< p.transpose() << std::endl;
	path.push_back( p );
      } // for i

      mBc.followTrajectory( path, maxAccel, maxVel );
      
    } else {
      SNS_LOG( LOG_ERR, "Invalid path_dense msg \n" );
      return false;
    }

  } break;
  case ACH_TIMEOUT:
  case ACH_STALE_FRAMES:
  case ACH_CANCELED:
    break;
  default:
    SNS_LOG( LOG_ERR, "Failed ach_get: %s \n", ach_result_to_string(r) );
    
  } // switch r
}
