/**
 * @file perception_msgs.h
 * @brief Last modified: Tuesday, October 14th, 2014
 * @author A. Huaman Quispe <ahuaman3 at gatech dot edu>
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
    double table_coeffs[4];        
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




  /**
   * @struct sns_msg_box
   * @brief Define a container box (with hole in the middle)
   */
  struct sns_msg_box {
    struct sns_msg_header header;
    double p[3]; // Position (x,y,z)
    double q[4]; // Rotation (quaternion in Eigen order)
    double s[4]; // Size (a,b,c,width)
  };

  /**
   * @struct sns_msg_rgb_img
   * @brief Define a RGB image
   */
  struct sns_msg_rgb_img {
    uint32_t width;
    uint32_t height;
    struct {
      uint8_t x;
      uint8_t y;
      uint8_t z;
    } u[1];
  };

  static inline size_t sns_msg_rgb_img_size_tn( uint32_t _width, uint32_t _height ) {
    static const struct sns_msg_rgb_img* msg;
    return sizeof( *msg ) + sizeof( msg->u[0] ) + sizeof( msg->u[0] )*_width*_height;
  }

  static inline size_t sns_msg_rgb_img_size( const struct sns_msg_rgb_img* _msg ) {
    return sns_msg_rgb_img_size_tn( _msg->width, _msg->height );
  }

  struct sns_msg_rgb_img* sns_msg_rgb_img_alloc( uint32_t _width, uint32_t _height );
  void sns_msg_rgb_img_dump( FILE* _out,
                             const struct sns_msg_rgb_img* _msg );

 
  

#ifdef __cplusplus
}
#endif
