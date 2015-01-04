/**
 * @file gatekeeper.cpp
 * @brief In charge of reading arm trajectories from client
 */
#include <sns.h>
#include <unistd.h>
#include <ach.h>

#include "base_control.h"

/** Left arm */
ach_channel_t la_state_chan;
ach_channel_t la_ref_chan;
ach_channel_t la_traj_chan;
BaseControl bc_left;

std::list<Eigen::VectorXd> la_path;

/** Right arm */
ach_channel_t ra_state_chan;
ach_channel_t ra_ref_chan;
ach_channel_t ra_traj_chan;
BaseControl bc_right;

std::list<Eigen::VectorXd> ra_path;

/** Additional data */
double mFreq = 100.0;
double mDt = 1.0 / mFreq;

double mMaxAccel = 0.1;
double mMaxVel = 0.18;

struct timespec t_now;
struct timespec t_timeout;


/** Polling function */
bool poll_traj_chan( ach_channel_t* _chan,
		     std::list<Eigen::VectorXd> &_path );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  Eigen::VectorXd maxVel, maxAccel;
  
  // Open channels
  sns_chan_open( &la_state_chan, "state-left", NULL );
  sns_chan_open( &la_ref_chan, "ref-left", NULL );
  sns_chan_open( &la_traj_chan, "traj-left", NULL );
  
  sns_chan_open( &ra_state_chan, "state-right", NULL );
  sns_chan_open( &ra_ref_chan, "ref-right", NULL );
  sns_chan_open( &ra_traj_chan, "traj-right", NULL );

  // Set channels
  bc_left.set_channels( &la_ref_chan, &la_state_chan );
  bc_right.set_channels( &ra_ref_chan, &ra_state_chan );
  

  // Loop waiting for traj
  while( true ) {
    if( poll_traj_chan( &la_traj_chan, la_path ) == true ) {
      int n_dof = (*la_path.begin()).size();
      bc_left.set_numJoints( n_dof );
      maxVel = mMaxVel*Eigen::VectorXd::Ones( n_dof );
      maxAccel = mMaxAccel*Eigen::VectorXd::Ones( n_dof );
      bc_left.followTrajectory( la_path, maxAccel, maxVel );
    }

    if( poll_traj_chan( &ra_traj_chan, ra_path ) == true ) {
      int n_dof = (*ra_path.begin()).size();
      bc_right.set_numJoints( n_dof );
      maxVel = mMaxVel*Eigen::VectorXd::Ones( n_dof );
      maxAccel = mMaxAccel*Eigen::VectorXd::Ones( n_dof );
      bc_right.followTrajectory( ra_path, maxAccel, maxVel );

    }

    usleep(1e6*mDt);
  }

  return 0;
}


/**
 * @function poll_traj_chan
 * @brief Checks if a message is received through traj_chan
 */
bool poll_traj_chan( ach_channel_t* _chan,
		     std::list<Eigen::VectorXd> &_path ) {

  _path.resize(0);
  
  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_now ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return 0;
  }

  t_timeout = sns_time_add_ns( t_now, 1000*1000*1 );
  
  size_t frame_size;
  void* buf = NULL;
  ach_status_t r = sns_msg_local_get( _chan,
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
            
      printf("\t * [INFO] Received trajectory with %d points and %d dofs \n", n_steps, n_dofs );
      

      int counter = 0;
      for( int i = 0; i < n_steps; ++i ) {
	Eigen::VectorXd p(n_dofs);
	for( int j = 0; j < n_dofs; ++j ) {
	  p(j) = msg->x[counter];
	  counter++;
	}
	std::cout << "P["<<i<<"]: "<< p.transpose() << std::endl;
	_path.push_back( p );
      } // for i

      return true;
      
    } else {
      SNS_LOG( LOG_ERR, "\t [ERROR] Invalid path_dense msg \n" );
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

  return false;
}
