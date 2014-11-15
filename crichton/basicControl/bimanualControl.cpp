/**
 * @file bimanualControl.cpp
 */

#include "bimanualControl.h"

/**
 * @function BimanualControl
 * @brief Constructor
 */
BimanualControl::BimanualControl() {
  mFreq = 100.0; // Hz
  mDt = 1.0 / mFreq;
  mDq_thresh = 0.1;
}

/**
 * @function setChannels
 * @brief Set arm and gripper channels
 */
bool BimanualControl::setChannels( ach_channel_t* _arm_state_chan[NUM_ARMS],
				   ach_channel_t* _arm_ref_chan[NUM_ARMS],
				   ach_channel_t* _hand_state_chan[NUM_ARMS],
				   ach_channel_t* _hand_ref_chan[NUM_ARMS] ) {

  // For left and right
  for( int i = 0; i < NUM_ARMS; ++i ) {
    mUni[i].setChannels( _arm_state_chan[i],
			 _arm_ref_chan[i],
			 _hand_state_chan[i],
			 _hand_ref_chan[i] );

  }
  
}


/**
 * @function update
 * @brief Read the latest state of the motors
 */
bool BimanualControl::update() {

  if( clock_gettime( ACH_DEFAULT_CLOCK, &now ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return 0;
  }
  struct timespec timeout = sns_time_add_ns( now, 1000*1000*1 );

  // Both arms (not grippers by now)
  bool left = mUni[LEFT_BIM].update();
  bool right = mUni[RIGHT_BIM].update();
  return left && right;
}

/**
 * @function getStates
 * @brief Return states
 */
void BimanualControl::getStates( Eigen::VectorXd _q_arm[NUM_ARMS],
				 Eigen::VectorXd _dq_arm[NUM_ARMS],
				 Eigen::VectorXd _q_hand[NUM_ARMS],
				 Eigen::VectorXd _dq_hand[NUM_ARMS] ) {

  for( size_t i = 0; i < NUM_ARMS; ++i ) {
    mUni[i].getStates( _q_arm[i], _dq_arm[i],
		       _q_hand[i], _dq_hand[i] );
  }

}


/**
 * @function followPath
 */
bool BimanualControl::followArmPath( int _side, 
				     const std::vector<Eigen::VectorXd> &_traj) {

  if( _side != LEFT_BIM && _side != RIGHT_BIM ) { 
    printf("\t * Side is neither left or right! \n. Exit and no move the robot \n");
    return false;
  } else {
    mUni[_side].followArmPath( _traj );
    return true;
  }

}

/**
 * @function followTrajectory
 */
bool BimanualControl::followTrajectory( int _side,
					const std::list<Eigen::VectorXd> &_path,
					const Eigen::VectorXd &_maxAccel,
					const Eigen::VectorXd &_maxVel ) {

  if( _side != LEFT_BIM && _side != RIGHT_BIM ) { 
    printf("\t * Side is neither left or right! \n. Exit and no move the robot \n");
    return false;
  } else {
    mUni[_side].followTrajectory( _path, _maxAccel, _maxVel );
    return true;
  }

}

/**
 * @function setHandConfig
 */
bool BimanualControl::setHandConfig( int _side,
				     const Eigen::VectorXd &_config,
				     double dt ) {
  if( _side != LEFT_BIM && _side != RIGHT_BIM ) { 
    printf("\t * Side is neither left or right! \n. Exit and no move the robot \n");
    return false;
  } else {
    mUni[_side].setHandConfig( _config, dt );
    return true;
  }


}
