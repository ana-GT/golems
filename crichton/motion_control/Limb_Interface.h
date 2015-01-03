/**
 * @file Limb_Interface.h
 */
#pragma once

#include "Arm_Interface.h"
#include "Hand_Interface.h"

class Limb_Interface  {

 public:
  Limb_Interface();
  ~Limb_Interface();

  void set_numJoints( int _num_arm_joints,
		      int _num_hand_joints );
  void set_hand_channels( ach_channel_t* _hand_state_chan,
			  ach_channel_t* _hand_output_chan );
  void set_arm_channels( ach_channel_t* _arm_state_chan,
			 ach_channel_t* _arm_output_chan );

  bool update();
  bool update_arm();
  bool update_hand();
  
  void get_arm_state( Eigen::VectorXd &_q,
		      Eigen::VectorXd &_dq );
  void get_hand_state( Eigen::VectorXd &_q,
		       Eigen::VectorXd &_dq );

  bool follow_arm_trajectory( const std::list<Eigen::VectorXd> &_path );
  bool go_hand_configuration( const Eigen::VectorXd &_config,
			      double _dt );
 protected:
  Arm_Interface mAi;
  Hand_Interface mHi;  
};
