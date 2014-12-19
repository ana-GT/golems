#include "perception_msgs.h"

/**
 * @function sns_msg_segmented_cloud_alloc
 * @brief Outputs allocated space for a pointer to a segmented_cloud message
 */
struct sns_msg_segmented_cloud* sns_msg_segmented_cloud_alloc( uint64_t _n) {
  return sns_msg_segmented_cloud_heap_alloc( (uint32_t) _n );
}

/**
 * @function sns_msg_segmented_cloud_dump
 * @brief Dumps information of a segmented_cloud message
 */
void sns_msg_segmented_cloud_dump( FILE* _out,
				   const struct sns_msg_segmented_cloud *_msg ) {

  fprintf( _out, "n_clusters: %d \n", _msg->n_clusters );
  fprintf( _out, "n_points: %d \n", _msg->header.n );
  fprintf( _out, "table coeffs: %f %f %f %f \n", _msg->table_coeffs[0], _msg->table_coeffs[1], _msg->table_coeffs[2], _msg->table_coeffs[3] );
  uint32_t i;
  for( i = 0; i < _msg->header.n; ++i ) {
    fprintf( _out, "\t %f %f %f - cluster: %d \n", _msg->u[i].x, 
	     _msg->u[i].y,
	     _msg->u[i].z ); //,
	     //_msg->u[i].cluster );
  }
  fprintf( _out, "\n" );
}


/**
 * @function sns_msg_segmented_rgb_img_alloc
 * @brief Outputs allocated space for a pointer to a rgb_img
 */
struct sns_msg_rgb_img* sns_msg_rgb_img_alloc( uint32_t _width, uint32_t _height ) {

    size_t size = sns_msg_rgb_img_size_tn( _width, _height );
    
    struct sns_msg_rgb_img* msg = (struct sns_msg_rgb_img*) malloc( size );
    memset( msg, 0, sizeof(*msg) );
    
    msg->width = _width;
    msg->height = _height;

    return msg;
}

/**
 * @function sns_msg_rgb_img_dump
 * @brief Dumps information of a rgb_img message
 */
void sns_msg_rgb_img_dump( FILE* _out,
                           const struct sns_msg_rgb_img* _msg ) {

    fprintf( _out, "Image width: %d \n", _msg->width );
    fprintf( _out, "Image height: %d \n", _msg->height );
}
