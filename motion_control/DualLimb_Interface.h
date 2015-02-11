/**
 * @file DualLimb_Interface.h
 */
#pragma once

#include "Limb_Interface.h"
#include "Arm_Interface.h"
#include "Hand_Interface.h"
#include "msgs/bimanual_msgs.h"

class DualLimb_Interface  {
  
 public:
  DualLimb_Interface();
  ~DualLimb_Interface();

  void set_numJoints( int _num_arm_joints,
		      int _num_hand_joints );
  void set_hand_channels( ach_channel_t* _hand_left_state_chan,
			  ach_channel_t* _hand_right_state_chan,
			  ach_channel_t* _hand_output_chan );
  void set_arm_channels( ach_channel_t* _arm_left_state_chan,
			 ach_channel_t* _arm_right_state_chan,
			 ach_channel_t* _arm_output_chan );

  bool update();
  bool update( int _i );
  bool update_arm( int _i );
  bool update_hand( int _i );
  
  void get_arm_state( int _i,
		      Eigen::VectorXd &_q,
		      Eigen::VectorXd &_dq );
  void get_hand_state( int _i,
		       Eigen::VectorXd &_q,
		       Eigen::VectorXd &_dq );

  bool follow_arm_trajectory( int _i,
			      const std::list<Eigen::VectorXd> &_path );
  bool follow_dual_arm_trajectory( const std::list<Eigen::VectorXd> &_leftPath,
				   const std::list<Eigen::VectorXd> &_rightPath );
  
  bool go_hand_configuration( int i,
			      const Eigen::VectorXd &_config,
			      double _dt );
  bool go_dual_hand_configuration( const std::list<Eigen::VectorXd> &_leftPath,
				   const std::list<Eigen::VectorXd> &_rightPath );
 protected:
  Limb_Interface mLi[2];
  struct timespec mNow;
  ach_channel_t* mChan_bimanualArm;
  ach_channel_t* mChan_bimanualHand;
};
