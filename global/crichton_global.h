/**
 * @file channel_names
 * @brief Common channel names used in Crichton's experiments
 * @author A. Huaman Quispe
 * @date 2015/01/02
 */
#pragma once

#include <string>
#include <Eigen/Geometry>

static const std::string gPoint_chan_name("point_chan");
static const std::string gObj_param_chan_name("obj_param_chan");
extern Eigen::Isometry3d gTrk[2];

void loadGlobals_golems();
