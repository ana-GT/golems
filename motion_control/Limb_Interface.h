/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Ana C. Huaman Quispe <ahuaman3@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
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
