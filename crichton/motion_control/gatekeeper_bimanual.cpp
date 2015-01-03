/**
 * @file gatekeeper.cpp
 * @brief In charge of reading arm trajectories from client
 */
#include <sns.h>
#include <unistd.h>
#include <ach.h>

#include "msgs/bimanual_msgs.h"
#include "base_dual_control.h"

ach_channel_t bimanual_chan;

/** Left arm */
ach_channel_t la_state_chan;
ach_channel_t la_ref_chan;

std::list<Eigen::VectorXd> la_path;

/** Right arm */
ach_channel_t ra_state_chan;
ach_channel_t ra_ref_chan;

std::list<Eigen::VectorXd> ra_path;

/** Dual control */
BaseDualControl bdc;

/** Additional data */
double mFreq = 100.0;
double mDt = 1.0 / mFreq;

double mMaxAccel = 0.1;
double mMaxVel = 0.18;

struct timespec t_now;
struct timespec t_timeout;
int mode;

/** Polling function */
bool poll_bimanual_chan( ach_channel_t* _chan,
			 int &_mode,
			 std::list<Eigen::VectorXd> &_left_path,
			 std::list<Eigen::VectorXd> &_right_path );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  Eigen::VectorXd maxVel, maxAccel;
  
  // Open channels
  sns_chan_open( &bimanual_chan, "bimanual_chan", NULL );
  
  sns_chan_open( &la_state_chan, "state-left", NULL );
  sns_chan_open( &la_ref_chan, "ref-left", NULL );
  
  sns_chan_open( &ra_state_chan, "state-right", NULL );
  sns_chan_open( &ra_ref_chan, "ref-right", NULL );

  // Set channels
  bdc.set_channels( 0, &la_ref_chan, &la_state_chan );
  bdc.set_channels( 1, &ra_ref_chan, &ra_state_chan );
  
  // Loop waiting for traj
  while( true ) {
    if( poll_bimanual_chan( &bimanual_chan,
			    mode,
			    la_path,
			    ra_path ) == true ) {
      int n_dof;
      if( mode == 0 ) { n_dof = (*la_path.begin()).size(); }      
      else if( mode == 1 ) { n_dof = (*ra_path.begin()).size(); }
      else if( mode == 2 ) { n_dof = (*la_path.begin()).size(); } // Same as right
      bdc.set_numJoints( n_dof );
      maxVel = mMaxVel*Eigen::VectorXd::Ones( n_dof );
      maxAccel = mMaxAccel*Eigen::VectorXd::Ones( n_dof );

      if( mode == 0 ) {
	printf("Follow left \n");
	bdc.followTrajectory( 0, la_path, maxAccel, maxVel );
      } else if( mode == 1 ) {
	printf("Follow right \n");
	bdc.followTrajectory( 1, ra_path, maxAccel, maxVel );
      } else if( mode == 2 ) {
	printf("Follow dual path \n");
	bdc.followDualTrajectory( la_path,
				  ra_path,
				  maxAccel, maxVel );
      } else {
	printf("Mode not valid \n");
      }
    }

    usleep(1e6*mDt);
  }

  return 0;
}


/**
 * @function poll_bimanual_chan
 * @brief Checks if a message is received through traj_chan
 */
bool poll_bimanual_chan( ach_channel_t* _chan,
			 int &_mode,
			 std::list<Eigen::VectorXd> &_leftPath,
			 std::list<Eigen::VectorXd> &_rightPath ) {

  _leftPath.resize(0);
  _rightPath.resize(0);
    
  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_now ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return false;
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
    printf("Gotten something \n");
    struct sns_msg_bimanual* msg = (struct sns_msg_bimanual*) buf;
    if( frame_size == sns_msg_bimanual_size(msg) ) {
      
      // Store trajectory
      int n_dofs, n_steps_left, n_steps_right, mode;
      n_dofs = msg->n_dof;
      n_steps_left = msg->n_steps_left;
      n_steps_right = msg->n_steps_right;
      _mode = msg->mode;
      printf("\t * [INFO] Received trajectory in mode (%d) with %d left and %d right points and %d dofs \n", _mode, n_steps_left, n_steps_right, n_dofs );
      
      if( _mode == 0 ) {
	int counter = 0;
	for( int i = 0; i < n_steps_left; ++i ) {
	  Eigen::VectorXd p(n_dofs);
	  for( int j = 0; j < n_dofs; ++j ) {
	    p(j) = msg->x[counter];
	    counter++;
	  }
	  std::cout << "Pl["<<i<<"]: "<< p.transpose() << std::endl;
	  _leftPath.push_back( p );
	} // for i
      } else if( _mode == 1 ) {
	int counter = 0;
	for( int i = 0; i < n_steps_right; ++i ) {
	  Eigen::VectorXd p(n_dofs);
	  for( int j = 0; j < n_dofs; ++j ) {
	    p(j) = msg->x[counter];
	    counter++;
	  }
	  std::cout << "Pr["<<i<<"]: "<< p.transpose() << std::endl;
	  _rightPath.push_back( p );
	} // for i
	
      } else if( _mode == 2 ) {

	int counter = 0;
	for( int i = 0; i < n_steps_left; ++i ) {
	  Eigen::VectorXd p(n_dofs);
	  for( int j = 0; j < n_dofs; ++j ) {
	    p(j) = msg->x[counter];
	    counter++;
	  }
	  std::cout << "Pl["<<i<<"]: "<< p.transpose() << std::endl;
	  _leftPath.push_back( p );
	} // for i

	counter = n_steps_left * n_dofs;
	for( int i = 0; i < n_steps_right; ++i ) {
	  Eigen::VectorXd p(n_dofs);
	  for( int j = 0; j < n_dofs; ++j ) {
	    p(j) = msg->x[counter];
	    counter++;
	  }
	  std::cout << "Pr["<<i<<"]: "<< p.transpose() << std::endl;
	  _rightPath.push_back( p );
	} // for i
	
	

      } else {
	printf("Error, mode incorrect \n");
	return false;
      }
      
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
