/**
 * @file bimanual_msgs.cpp
 * @date 2014/12/23
 */

#include "bimanual_msgs.h"


/**
 * @function sns_msg_bimanual_alloc
 * @brief Returns a path msg with enough space allocated for n_steps (left and right) and n_dof
 */
struct sns_msg_bimanual *sns_msg_bimanual_alloc( uint16_t _n_steps_left,
						 uint16_t _n_steps_right,
						 uint8_t _n_dof ) {
  size_t size = sns_msg_bimanual_size_tn( _n_steps_left,
					  _n_steps_right,
					  _n_dof );
  struct sns_msg_bimanual *msg = (struct sns_msg_bimanual*) malloc( size );
  memset( msg, 0, sizeof(*msg) );

  msg->n_dof = _n_dof;
  msg->n_steps_left = _n_steps_left;
  msg->n_steps_right = _n_steps_right;
  
  return msg;
}

/**
 * @function sns_path_dense_dump
 * @brief Outputs the path to a file (i.e. STDOUT) in a readable form
 */
void sns_msg_bimanual_dump( FILE* _out,
			    const struct sns_msg_bimanual *_msg ) {
  //dump_header( _out, &_msg->header, "path_ref" );

  uint16_t n_dof = _msg->n_dof;
  uint16_t n_steps_left = _msg->n_steps_left;
  uint16_t n_steps_right = _msg->n_steps_right;
  uint8_t mode = _msg->mode;
  
  fprintf( _out, "n_dof: %d \n", n_dof );
  fprintf( _out, "n_steps_left: %d \n", n_steps_left );
  fprintf( _out, "n_steps_right: %d \n", n_steps_right );
  fprintf( _out, "mode: %d \n", mode );

  uint32_t i, j;

  if( mode == 0 ) {

    fprintf(_out, "Left arm trajectory: \n");
    for( i = 0; i < _msg->n_steps_left; i++ ) {
      for( j = 0; j < n_dof; ++j ) {
	fprintf( _out, "\t %f", _msg->x[i*n_dof + j]);
      }
      fprintf( _out, "\n" );
    }

    
  } else if( mode == 1 ) {
  
    fprintf(_out, "Right arm trajectory: \n");
    for( i = 0; i < _msg->n_steps_right; i++ ) {
      for( j = 0; j < n_dof; ++j ) {
	fprintf( _out, "\t %f", _msg->x[i*n_dof + j]);
      }
      fprintf( _out, "\n" );
    }
  } else if( mode == 2 ) {
    fprintf(_out, "Dual mode. Left first then right \n");
    fprintf(_out, "Left arm trajectory: \n");
    for( i = 0; i < _msg->n_steps_left; i++ ) {
      for( j = 0; j < n_dof; ++j ) {
	fprintf( _out, "\t %f", _msg->x[i*n_dof + j]);
      }
      fprintf( _out, "\n" );
    }

    fprintf(_out, "Right arm trajectory: \n");
    int start_right = _msg->n_steps_left* n_dof; 
    for( i = start_right; i < start_right + _msg->n_steps_right; i++ ) {
      for( j = 0; j < n_dof; ++j ) {
	fprintf( _out, "\t %f", _msg->x[i*n_dof + j]);
      }
      fprintf( _out, "\n" );
    }

    
  }
}


