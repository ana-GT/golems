/**
 * @file BimanualControl.h
 * @brief Simple interface with different sequences to send info to arm through ACH
 */
#pragma once

#include <unistd.h>
#include <sns.h>
#include <ach.h>
#include <Eigen/Dense>

#include "unimanualControl.h"

enum ARM_SIDE {
  LEFT_BIM=0,
  RIGHT_BIM=1,
  NUM_ARMS=2
};

/**
 * @class BimanualControl
 */
class BimanualControl : public UnimanualControl {
  
 public:
  BimanualControl();
  bool setChannels( ach_channel_t* _arm_state_chan[NUM_ARMS],
		    ach_channel_t* _arm_ref_chan[NUM_ARMS],
		    ach_channel_t* _hand_state_chan[NUM_ARMS],
		    ach_channel_t* _hand_ref_chan[NUM_ARMS] );

  bool update();
  void getStates( Eigen::VectorXd _q_a[NUM_ARMS],
		  Eigen::VectorXd _dq_a[NUM_ARMS],
		  Eigen::VectorXd _q_h[NUM_ARMS],
		  Eigen::VectorXd _dq_h[NUM_ARMS] );
  
  bool followArmPath( int _side,
		      const std::vector<Eigen::VectorXd> &_traj);

  bool followTrajectory( int _side,
			 const std::list<Eigen::VectorXd> &_path,
			 const Eigen::VectorXd &_maxAccel,
			 const Eigen::VectorXd &_maxVel );

  bool setHandConfig( int _side,
		      const Eigen::VectorXd &_config,
		      double dt = 3 );

 private:  

  UnimanualControl mUni[2];

};
