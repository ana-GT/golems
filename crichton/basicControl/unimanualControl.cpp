/**
 * @file unimanualControl.cpp
 */

#include "unimanualControl.h"
#include "trajectories/Trajectory.h"
#include "trajectories/Path.h"

#include <fstream>

/**
 * @function UnimanualControl
 * @brief Constructor
 */
UnimanualControl::UnimanualControl() {
  mFreq = 100.0; // Hz
  mDt = 1.0 / mFreq;
  mDq_thresh = 0.1;
}

/**
 * @function setChannels
 * @brief Set arm and gripper channels
 */
bool UnimanualControl::setChannels( ach_channel_t* _arm_state_chan,
				    ach_channel_t* _arm_ref_chan,
				    ach_channel_t* _hand_state_chan,
				    ach_channel_t* _hand_ref_chan ) {

  mChan_arm_state = _arm_state_chan;
  mChan_arm_ref = _arm_ref_chan;
  
  mChan_hand_state = _hand_state_chan;
  mChan_hand_ref = _hand_ref_chan;
}


/**
 * @function update
 * @brief Read the latest state of the motors
 */
bool UnimanualControl::update() {

  if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return false;
  }
  struct timespec timeout = sns_time_add_ns( now, 1000*1000*5 );
  // Both arms (not grippers by now)
  int u_a = update_n( ARM_AXES, 
		      mArm_q, mArm_dq, 
		      mChan_arm_state, 
		      &timeout );

  int u_h = update_n( HAND_AXES, 
		      mHand_q, mHand_dq, 
		      mChan_hand_state, 
		      &timeout );
  aa_mem_region_local_release();
 
  return u_a && u_h;
}

/**
 * @function getStates
 * @brief Return states
 */
void UnimanualControl::getStates( Eigen::VectorXd &_q_arm,
				 Eigen::VectorXd &_dq_arm,
				 Eigen::VectorXd &_q_hand,
				 Eigen::VectorXd &_dq_hand ) {

  _q_arm.resize(ARM_AXES);
  _dq_arm.resize(ARM_AXES);

  _q_hand.resize(HAND_AXES);
  _dq_hand.resize(HAND_AXES);
  
  for( int i = 0; i < ARM_AXES; ++i ) {
    _q_arm(i) = mArm_q[i];
    _dq_arm(i) = mArm_dq[i];
  }
  for( int i = 0; i < HAND_AXES; ++i ) {
    _q_hand(i) = mHand_q[i];
    _dq_hand(i) = mHand_dq[i];
  }
}

/**
 * @function followTrajectory
 * @brief plot 'test.txt' using 1:2 with lines, '' using 1:3 with lines, '' using 1:4 with lines, '' using 1:5 with lines, '' using 1:6 with lines, '' using 1:7 with lines
 */
bool UnimanualControl::followTrajectory( const std::list<Eigen::VectorXd> &_path,
					 const Eigen::VectorXd &_maxAccel,
					 const Eigen::VectorXd &_maxVel ) {
  
  std::list<Eigen::VectorXd> path;
  path = _path;

  // Safety check
  Eigen::VectorXd current(ARM_AXES);
  printf("\t * Check if it is reasonable to follow this path \n");
  while( !update() ) {}
  for(int i =0; i < ARM_AXES; ++i ) { current(i) = mArm_q[i]; }
  
  if( ( *(path.begin()) - current).norm() > mDq_thresh ) {
    
    printf("\t * [WARNING] First point of trajectory is too far from initial point in path. \n");
    std::cout << "* Current state: " << current.transpose() << std::endl;
    std::cout << "* First trajectory point: "<<(*path.begin()).transpose() << std::endl;
    
    if( ( *(path.begin()) - current).norm() < 2*mDq_thresh ) {
      printf("\t * [FIX] Adding first point of current state \n");
      path.push_front( current );
      return true;
    }
    else {
      printf("\t * [NO FIX] Too far (more than twice step) \n");
      return false;
    }
  } else {
    printf("\t * [DEBUG] First point of path and state is all right \n");
  }

  
  // Create trajectory
  Trajectory trajectory( Path(path, 0.1), _maxVel, _maxAccel );
  trajectory.outputPhasePlaneTrajectory();

  if( trajectory.isValid() == false ) { 
    printf("\t [ERROR] Trajectory is not valid. \n");
    return false; 
  }

  double duration = trajectory.getDuration();
  double start_time;
  double current_time;
  Eigen::VectorXd vel_cmd;
  printf( "\t [DEBUG] Duration of trajectory: %f \n", duration );
  

  // Loop
  while( !update() ) {}

  start_time = now.tv_sec + (now.tv_nsec)/(1.0e9);
  current_time = start_time;


  // Send velocity commands
  while( current_time < start_time + duration ) {

    // Get current time and state
    while( !update() ) {}

    current_time = now.tv_sec + (now.tv_nsec)/(1.0e9);

    // Get velocity command
    vel_cmd = trajectory.getVelocity(current_time - start_time);
    std::cout << "["<< (current_time - start_time) << "]:"<< vel_cmd.transpose() << std::endl;
    // Send command to robot    
/*
    if( !control_n( ARM_AXES, vel_cmd.data(), mDt, mChan_arm_ref, SNS_MOTOR_MODE_VEL ) ) {
      printf("[followTrajectory] Sending velocity message did not go well \n");
      return false;
    }
*/    

    // Sleep and clean up
    usleep ((useconds_t)(1e6*mDt) ); 
    aa_mem_region_local_release();
  } // end while

  printf("\t * [DEBUG] Finished and sending zero vel \n");
  
  double q[ARM_AXES]; for( int i = 0; i < ARM_AXES; ++i ) { q[i] = 0; }
  control_n( ARM_AXES, q, mDt, mChan_arm_ref, SNS_MOTOR_MODE_VEL );

  return true;
}


/**
 * @function followPath
 */
void UnimanualControl::followArmPath( const std::vector<Eigen::VectorXd> &_traj) {

  // Start behavior
  size_t key_idx = 0;
  Eigen::VectorXd current(ARM_AXES);
  Eigen::VectorXd currentGoal(ARM_AXES);
  double dist;
  
  printf("\t * Check if it is reasonable to follow this path \n");
  while( !update_n(ARM_AXES, mArm_q, 
		   mArm_dq, 
		   mChan_arm_state, NULL ) ) {}
  for(int i =0; i < ARM_AXES; ++i ) { current(i) = mArm_q[i]; }
  if( (_traj[0] - current).norm() > mDq_thresh ) {
    printf("\t * FIRST POINT OF TRAJ IS TOO FAR FROM INITIAL PLAN POINT. ARE YOU SURE THIS IS ALL RIGHT? \n");
    return;
  } else {
    printf("\t * SHOULD BE GOOD TO GO \n");
  }
  
  
  printf("\t * Start following trajectory of size: %d \n", _traj.size() );
  bool stillFollow = true;
  
  while(stillFollow) {
    // Get the current state
    while( !update_n(ARM_AXES, mArm_q, 
		     mArm_dq, 
		     mChan_arm_state, NULL ) ) {}
    for(int i =0; i < ARM_AXES; ++i ) { current(i) = mArm_q[i]; }
    
    // Check if the target point keypoint is reached, if so, give command 
    // for the next one
    currentGoal = _traj[key_idx];
    dist = ( currentGoal - current ).norm();
    if( dist < mDq_thresh ) {
      if( key_idx == _traj.size() - 1 ) {
	stillFollow = false;
      }
      key_idx = std::min( _traj.size() - 1, key_idx + 1 );
      printf("Following point %d \n", key_idx );
      currentGoal = _traj[key_idx];
    }
    

    // Tell the robot to go to the current goal
    if( !control_n( ARM_AXES, currentGoal.data(), mDt, mChan_arm_ref ) ) {
		printf("Sending motor message did not go well! \n");
    }
    
    // Sleep and clean up
    if( mFreq > 0 ) { usleep ((useconds_t)(1e6*mDt) ); } 
    aa_mem_region_local_release();

  } // end while

  printf("\t * Done! \n");

}

/**
 * @function setHandConfig
 */
bool UnimanualControl::setHandConfig( const Eigen::VectorXd &_config,
				      double _dt ) {
  Eigen::VectorXd q; q = _config;
  return control_n( HAND_AXES, q.data(), _dt, mChan_hand_ref );
}

