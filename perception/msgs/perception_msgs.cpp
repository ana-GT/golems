#include "perception_msgs.h"

struct sns_msg_segmented_cloud* sns_msg_segmented_cloud_alloc( uint64_t _n) {
  return sns_msg_segmented_cloud_heap_alloc( (uint32_t) _n );
}

void sns_msg_segmented_cloud_dump( FILE* _out,
				   const struct sns_msg_segmented_cloud *_msg ) {

  fprintf( _out, "n_clusters: %d \n", _msg->n_clusters );
  fprintf( _out, "n_points: %d \n", _msg->header.n );
  
  uint32_t i;
  for( i = 0; i < _msg->header.n; ++i ) {
    fprintf( _out, "\t %f %f %f - cluster: %d \n", _msg->u[i].x, 
	     _msg->u[i].y,
	     _msg->u[i].z ); //,
	     //_msg->u[i].cluster );
  }
  fprintf( _out, "\n" );
}
