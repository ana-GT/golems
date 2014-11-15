/**
 * @file UnimanualControl.h
 * @brief Simple interface with different sequences to send info to arm through ACH
 */
#pragma once

#include "basicControl.h"
#include <list>

/**
 * @class UnimanualControl
 */
class UnimanualControl: public BasicControl {
  
 public:

  UnimanualControl();
  bool setChannels( ach_channel_t* _arm_state_chan,
		    ach_channel_t* _arm_ref_chan,
		    ach_channel_t* _hand_state_chan,
		    ach_channel_t* _hand_ref_chan );

  bool update();
  void getStates( Eigen::VectorXd &_q_a,
		  Eigen::VectorXd &_dq_a,
		  Eigen::VectorXd &_q_h,
		  Eigen::VectorXd &_dq_h );
  
  bool followTrajectory( const std::list<Eigen::VectorXd> &_path,
			 const Eigen::VectorXd &_maxAccel,
			 const Eigen::VectorXd &_maxVel );
		      
  void followArmPath( const std::vector<Eigen::VectorXd> &_traj);

  bool setHandConfig( const Eigen::VectorXd &_config,
		      double dt = 3 );


 private:  

  ach_channel_t* mChan_arm_state;
  ach_channel_t* mChan_arm_ref;

  ach_channel_t* mChan_hand_state;
  ach_channel_t* mChan_hand_ref;


  double mArm_q[ARM_AXES];
  double mArm_dq[ARM_AXES];

  double mHand_q[HAND_AXES];
  double mHand_dq[HAND_AXES];

};
