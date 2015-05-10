/**
 * @file control_msgs.h
 * @brief Last modified: May, 5th, 2014
 * @author A. Huaman Quispe <ahuaman3 at gatech dot edu>
 */
#pragma once


#include <sns.h>


enum GATEKEEPER_MSG_TYPES {
  HAND_LEFT_END = 0,
  HAND_RIGHT_END = 1,
  HAND_BOTH_END = 2,
  ARM_LEFT_END = 3,
  ARM_RIGHT_END = 4,
  ARM_BOTH_END = 5
};
  
  /**
   * @struct gatekeeper_msg
   * @brief Msg letting receiver know that trajectory is finished being executed
   */
  struct gatekeeper_msg {
    struct sns_msg_header header;    
    uint8_t type;
    bool state;
  };
