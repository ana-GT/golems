/**
 * @file server_msgs.h
 * @brief Last modified: Tuesday, April 21st, 2015
 * @author A. Huaman Quispe <ahuaman3 at gatech dot edu>
 */
#pragma once

#include <sns.h>
#include <global/fsa_data.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct sns_msg_server1
 * @brief Type and an integer data
 */
struct sns_msg_server_1 {

struct sns_msg_header header;
  uint8_t task_type; /** Task type */
  uint8_t msg_type; /** Msg type (i.e. start, stop) */
  uint8_t data_int1;
  uint8_t data_int2;

};

/**
 * @struct sns_msg_process1
 * @brief Type and an integer data
 */
struct sns_msg_process_1 {

struct sns_msg_header header;
uint8_t task_type; /** Task type */
uint8_t msg_type; 
uint8_t data_int;
char data_str[255];
};


#ifdef __cplusplus
}
#endif
