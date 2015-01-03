/**
 * @file Limb_Interface.cpp
 */
#include "Limb_Interface.h"


/**
 * @function Limb_Interface
 */
Limb_Interface::Limb_Interface() {

}

Limb_Interface::~Limb_Interface() {

}

/**
 * @function set_numJoints
 */
void Limb_Interface::set_numJoints( int _num_arm_joints,
				    int _num_hand_joints ) {
  mAi.set_numJoints( _num_arm_joints );
  mHi.set_numJoints( _num_hand_joints );
}


/**
 * @function set_hand_channels
 */
void Limb_Interface::set_hand_channels( ach_channel_t* _hand_state_chan,
					ach_channel_t* _hand_output_chan ) {

  mHi.set_channels( _hand_state_chan,
		    _hand_output_chan );
}

/**
 * @function set_arm_channels
 */
void Limb_Interface::set_arm_channels( ach_channel_t* _arm_state_chan,
				       ach_channel_t* _arm_output_chan ) {
    mAi.set_channels( _arm_state_chan,
		      _arm_output_chan );
}

/**
 * @function update_arm
 */
bool Limb_Interface::update_arm() {
  return mAi.update();
}

/**
 * @function update_hand
 */
bool Limb_Interface::update_hand() {
  return mHi.update();
}

/**
 * @function update
 */
bool Limb_Interface::update() {
  return update_arm() && update_hand();
}

void Limb_Interface::get_arm_state( Eigen::VectorXd &_q,
				    Eigen::VectorXd &_dq ) {

  mAi.get_state( _q, _dq );
}

void Limb_Interface::get_hand_state( Eigen::VectorXd &_q,
				     Eigen::VectorXd &_dq ) {
  mHi.get_state( _q, _dq );
}

/**
 * @function follow_arm_trajectory
 */
bool Limb_Interface::follow_arm_trajectory( const std::list<Eigen::VectorXd> &_path ) {

  return mAi.followTrajectory( _path ); 
}

/**
 * @function go_hand_configuration
 */
bool Limb_Interface::go_hand_configuration( const Eigen::VectorXd &_config,
					    double _dt ) {

  return mHi.goToConfiguration( _config,
				_dt );
}
