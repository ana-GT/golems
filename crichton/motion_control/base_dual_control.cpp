/**
 * @file base_dual_control.cpp
 */
#include "base_dual_control.h"

#include "trajectories/Trajectory.h"
#include "trajectories/Path.h"

/**
 * @function BaseDualControl
 */
BaseDualControl::BaseDualControl() {
  
  mFreq = 100;
  mDt = 1.0 / mFreq;
  mDq_thresh = 0.1;
  
  mMaxDev = 0.1;
  mVALID_NS = (int64_t)((1000000000) / 5);
  
}

/**
 * @function ~BaseDualControl
 */
BaseDualControl::~BaseDualControl() {}

/**
 * @function set_numJoints
 */
void BaseDualControl::set_numJoints( int _N ) {

  mN = _N;
  
  for( size_t i = 0; i < 2; ++i ) {
    mBc[i].set_numJoints( mN );
  }
  
}

/**
 * @function set_channels
 */
void BaseDualControl::set_channels( int _i,
				    ach_channel_t* _ref_chan,
				    ach_channel_t* _state_chan ) {

  if( _i < 0 || _i >= 2 ) { printf("Error, index for channel is wrong \n"); }
  mBc[_i].set_channels( _ref_chan,
			_state_chan );
}

/**
 * @function update
 * @brief TODO
 */
bool BaseDualControl::update() {

  if( clock_gettime( ACH_DEFAULT_CLOCK, &mNow ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return false;
  }
  
  return mBc[0].update() && mBc[1].update();
}

/**
 * @function followTrajectory
 */
bool BaseDualControl::followTrajectory( int _side,
					const std::list<Eigen::VectorXd> &_path,
					const Eigen::VectorXd &_maxAccel,
					const Eigen::VectorXd &_maxVel ) {

  return mBc[_side].followTrajectory( _path, _maxAccel, _maxVel ); 
}

/**
 * @function followDualTrajectory
 */
bool BaseDualControl::followDualTrajectory( const std::list<Eigen::VectorXd> &_pathLeft,
					    const std::list<Eigen::VectorXd> &_pathRight,
					    const Eigen::VectorXd &_maxAccel,
					    const Eigen::VectorXd &_maxVel ) {

  std::list<Eigen::VectorXd> path[2];
  path[0] = _pathLeft;
  path[1] = _pathRight;
  
  // Safety check
  while( !this->update() ) {}

  for( int i = 0; i < 2; ++i ) {
    if( ( *(path[i].begin()) - mBc[i].get_q() ).norm() > mDq_thresh ) {
      printf("\t [followTrajectory] First point of trajectory [%d] is too far from init path point \n", i);
      return false;
    }
  }
  
  // Create trajectory
  Trajectory trajectory[2] = { Trajectory(Path(path[0], mMaxDev), _maxVel, _maxAccel),
			       Trajectory(Path(path[1], mMaxDev), _maxVel, _maxAccel) };
  double duration[2];
  double maxDuration;
  
  for( size_t i = 0; i < 2; ++i ) {
    //trajectory[i] = Trajectory( Path(path[i], mMaxDev), _maxVel, _maxAccel );
    trajectory[i].outputPhasePlaneTrajectory();

    if( !trajectory[i].isValid() ) {
      printf("\t [followTrajectory] ERROR: Trajectory is not valid \n");
      return false;
    }
   duration[i] = trajectory[i].getDuration();  
  }

  if( duration[0] > duration[1] ) { maxDuration = duration[0]; } else { maxDuration = duration[1]; }
  
  double start_time;
  double current_time;
  Eigen::VectorXd vel_cmd[2];
  Eigen::VectorXd zeros(mN); for( int i = 0; i < mN; ++i ) { zeros(i) = 0; }
  
  printf("\t [followDualTraj] DEBUG: Max. duration of trajectory: %f \n", maxDuration );

  // Update current state and time
  double tn;
  
  while( !this->update() ) {}
  
  start_time = mNow.tv_sec + (mNow.tv_nsec)/1.0e9;
  current_time = start_time;
  tn = current_time - start_time;

  // Send velocity commands
  while( tn < maxDuration ) {
    
    // Get current time and state
    while( !this->update() ) {}
    current_time = mNow.tv_sec + mNow.tv_nsec / (1.0e9);
    tn = current_time - start_time;   
    
    // Build velocity command
    for( int i = 0; i < 2; ++i ) {

      if( tn < duration[i] ) {
	vel_cmd[i] = trajectory[i].getVelocity( tn );
	/*
	  std::cout << "["<< tn << "]: "<< vel_cmd.transpose() << std::endl;
	  std::cout << "Expected position: "<< trajectory.getPosition(tn).transpose() << std::endl;
	  std::cout << "Current position: " << mq.transpose() << std::endl;
	  std::cout << "Current velocity: " << mdq.transpose() << std::endl;
	*/
	// Send command to robot    
	if( !mBc[i].control_n( mN, vel_cmd[i], 2*mDt, mBc[i].get_refChan(), SNS_MOTOR_MODE_VEL ) ) {
	  printf("\t[followTrajectory - %d] Sending vel msg error \n", i);
	  return false;
	}
      } else {
	mBc[i].control_n( mN, zeros, mDt, mBc[i].get_refChan(), SNS_MOTOR_MODE_VEL );
      }
    } // end for
    
    // Sleep and clean up
    usleep( (useconds_t)(1e6*mDt)) ;
    aa_mem_region_local_release();
    
    
  } // end while

  printf("\t * [trajectoryFollowing] DEBUG: Finish and sending zero vel for good measure \n");
  for( size_t i = 0; i < 2; ++i ) {
    mBc[i].control_n( mN, zeros, mDt, mBc[i].get_refChan(), SNS_MOTOR_MODE_VEL );
  }
  
  return true;
}
