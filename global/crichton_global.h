/**
 * @file channel_names
 * @brief Common channel names used in Crichton's experiments
 * @author A. Huaman Quispe
 * @date 2015/01/02
 */
#pragma once

#include <string>
#include <Eigen/Geometry>

enum ROBOT_SIDE {
  LEFT_MSG = 0,
  RIGHT_MSG = 1,
  NUM_SIDES_MSG = 2
};

enum GOAL_CONF {
  ARC_UP = 0,
  FRONTAL_FIST = 1,
  RESTING = 2,
  LOOKUP_SKY = 3,
  DEMANDING = 4,
  HUG = 5,
  TOTAL_CONF = 6
};

enum ACTION {
  START = 0,
  STOP = 1
};

static const std::string gPoint_chan_name("point_chan");
static const std::string gObj_param_chan_name("obj_param_chan");
static const std::string gServer2See_chan_name("server2see_chan");
static const std::string gSee2Server_chan_name("see2server_chan");
static const std::string gServer2Plan_chan_name("server2plan_chan");
static const std::string gPlan2Server_chan_name("plan2server_chan");
static const std::string gPicturesPath("/home/ana/Pictures");

extern Eigen::Isometry3d gTrk[2];
extern Eigen::VectorXd gKnownConfs[NUM_SIDES_MSG][TOTAL_CONF];



void loadGlobals_golems();
