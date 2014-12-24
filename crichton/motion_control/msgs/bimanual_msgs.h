/**
 * @file bimanual_msgs.h
 * @brief Last modified: Tuesday, December 23rd, 2014
 * @author A. Huaman Quispe <ahuaman3 at gatech dot edu>
 */
#pragma once


#include <sns.h>

#ifdef __cplusplus
extern "C" {
#endif
  
  /**
   * @struct sns_msg_bimanual
   * @brief
   */
  struct sns_msg_bimanual {
    
    struct sns_msg_header header;
    uint8_t n_dof; // Both arms have the same number of dofs
    uint8_t mode; // 0: left, 1: right, 2: both
    uint16_t n_steps_left; // Length of left trajectory
    uint16_t n_steps_right; // Length of right trajectory
    sns_real_t x[1]; // Trajectory left, count is: n_dof*(n_steps_left+n_steps_right);

  };

  /**
   * @function sns_msg_bimanual_size_tn
   * @brief Calculates the size of any message with its n_steps (left and right) and n_dof
   */
  static inline size_t sns_msg_bimanual_size_tn( uint16_t _n_steps_left,
						 uint16_t _n_steps_right,
						 uint8_t _n_dofs ) {
    static const struct sns_msg_bimanual *msg;
    return sizeof( *msg ) - sizeof( msg->x[0] ) + sizeof( msg->x[0] )*(_n_steps_left + _n_steps_right)*_n_dofs;
  }
  
  /**
   * @function sns_msg_bimanual_size
   * @brief Returns the size of the message according to its n_steps (left and right) and n_dof 
   */
  static inline size_t sns_msg_bimanual_size( const struct sns_msg_bimanual * _msg ) {
    return sns_msg_bimanual_size_tn( _msg->n_steps_left,
				     _msg->n_steps_right,
				     _msg->n_dof );
  }


  // DECLARATIONS
  struct sns_msg_bimanual *sns_msg_bimanual_alloc( uint16_t _n_steps_left,
						   uint16_t _nh_steps_Right,
						   uint8_t _n_dof );
  void sns_msg_bimanual_dump( FILE* _out,
			    const struct sns_msg_bimanual *_msg );
  

  


#ifdef __cplusplus
}
#endif
