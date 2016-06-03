/**
 * @file skeleton_tracker.h
 */

#pragma once

#include <NiTE.h>
#include <Eigen/Core>
#include <ach.h>
#include "msgs/perception_msgs.h"

enum JOINT_NAMES {
  HEAD=0,
  NECK=1,
  LEFT_SHOULDER=2,
  LEFT_ELBOW=3,
  LEFT_HAND=4,
  RIGHT_SHOULDER=5,
  RIGHT_ELBOW=6,
  RIGHT_HAND=7,
  TORSO=8,
  NUM_SKEL_JOINTS=9
};

class skeleton_tracker {

 public:
  skeleton_tracker();
  ~skeleton_tracker();

  openni::Status init( int argc, char* argv[] );
  void setChan( std::string _chan );
  void run();
  void update();
  void finalize();

  static std::string sJointNames[NUM_SKEL_JOINTS];
  
 private:
  
  nite::UserTracker* mUserTracker;
  openni::Device mDevice;
  ach_channel_t mJoints_chan;
  
  // Head/Neck/LShoulder/LElbow/LHand/RShoulder/RElbow/RHand/Torso
  static const int mNumJoints = NUM_SKEL_JOINTS;
  Eigen::Vector3d mJoints[mNumJoints];
  sns_msg_skel mMsg;
  
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
