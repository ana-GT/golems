/**
 * @file traj_test_receiver.cpp
 */

#include <stdio.h>
#include <basicControl/bimanualControl.h>
#include <string>
#include <sns/path.h>

bool check_traj_chan( struct timespec *ts );

double mDt = 0.01;
double mMaxVel = 0.18;
double mMaxAccel = 0.1;

BimanualControl mBc;
int arm_side;

struct timespec now;
struct timespec ts;

std::string traj_name[2];


ach_channel_t* traj_chan[2];


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  sns_init();
  sns_start();
  
  traj_name[0] = "test-left";
  traj_name[1] = "test-right";


  // Create channels  
  for( int i = 0; i < 2; ++i ) {
    traj_chan[i] = new ach_channel_t();
  }


  // Open & set channels
  for( int i = 0; i < 2; ++i ) {

    sns_chan_open( traj_chan[i], traj_name[i].c_str(), NULL );
  }
  
  // Loop until you hear a trajectory coming
  while(true) {
    
    if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
      SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
      printf("Did not get time right \n");
      return 0;
    }
    ts= sns_time_add_ns( now, 1000*1000*1 );

    check_traj_chan( &ts );
    usleep((1e6*mDt) );    
  }


  return 0;

}

/**
 * @function check_traj_chan
 */
bool check_traj_chan(struct timespec *ts) {



  std::list<Eigen::VectorXd> path;
  for( int c = 0; c < 2; ++c ) {
    size_t frame_size;
    void *buf = NULL;
    ach_status_t r = sns_msg_local_get( traj_chan[c], 
					&buf,
					&frame_size,
					ts, 
					ACH_O_LAST | (ts ? ACH_O_WAIT : 0 ) );
    switch(r) {
    
    case ACH_OK:
    case ACH_MISSED_FRAME: {
      struct sns_msg_path_dense *msg = (struct sns_msg_path_dense*)buf;
      if( frame_size == sns_msg_path_dense_size(msg) ) {
	
	// Save traj
	int n_dofs; int n_steps;
	n_dofs = msg->n_dof;
	n_steps = msg->n_steps;
	printf("Received trajectory with %d points and %d dofs \n", n_steps, n_dofs );

	Eigen::VectorXd maxVel; maxVel = mMaxVel*Eigen::VectorXd::Ones( n_dofs );
	Eigen::VectorXd maxAccel; maxAccel = mMaxAccel*Eigen::VectorXd::Ones( n_dofs );
	std::cout << "Max accel: "<< maxAccel.transpose() << std::endl;
	std::cout << "Max vel: "<< maxVel.transpose() << std::endl;

	int counter = 0;
	for( int i = 0; i < n_steps; ++i ) {
	  Eigen::VectorXd p(n_dofs);
	  for( int j = 0; j < n_dofs; ++j ) {
	    p(j) = msg->x[counter];
	    counter++;
	  }
	  path.push_back( p );
	}

	// Send trajectory
	printf("Received trajectory for arm %d. Executing \n", c);
	
	std::list<Eigen::VectorXd>::iterator it;
	int m = 0;
	for( it = path.begin(); it != path.end(); ++it ) {
	  std::cout << "P["<<m<<"]: "<< (*it).transpose() << std::endl;
	  m++;
	}
    std::cout << "Max accel: "<< maxAccel.transpose() << std::endl;
    std::cout << "Max vel: "<< maxVel.transpose() << std::endl;
	
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
  } // end for
  
}
