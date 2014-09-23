/**
 * @file perception_msgs.h
 */
#pragma once


#include <sns.h>

#ifdef __cplusplus
extern "C" {
#endif
  
  /**
   * @struct sns_msg_segmented_cloud
   * @brief
   */
  struct sns_msg_segmented_cloud {
    
    struct sns_msg_header header;
    uint8_t n_clusters; /** Number of clusters */
    uint8_t selected;
        
    struct {
      sns_real_t x;
      sns_real_t y;
      sns_real_t z;
      uint8_t cluster;
    } u[1];
  };
  
  // Create
  SNS_DEF_MSG_VAR( sns_msg_segmented_cloud, u );
  

  // DECLARATIONS
  struct sns_msg_segmented_cloud* sns_msg_segmented_cloud_alloc( uint64_t _n );
  
  void sns_msg_segmented_cloud_dump( FILE* _out,
				     const struct sns_msg_segmented_cloud *_msg );
  
#ifdef __cplusplus
}
#endif
