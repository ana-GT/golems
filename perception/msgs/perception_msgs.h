/**
 * @file perception_msgs.h
 * @brief Last modified: Wednesday, May 20th, 2015
 * @author A. Huaman Quispe <ahuaman3 at gatech dot edu>
 */
#pragma once

#include <sns.h>

#ifdef __cplusplus
extern "C" {
#endif

  /**
   * @struct sns_msg_marker_robot
   * @brief Carries the marker position of known joints with marker in the robot frame
   */
  struct sns_msg_marker_robot {
    struct sns_msg_header header;
    struct {
      double x[2]; // 0: left, 1: right
      double y[2];
      double z[2];
      double id[2]; // marker id
    } u[1];
  };

  SNS_DEF_MSG_VAR( sns_msg_marker_robot, u );
  
  // Declarations
  struct sns_msg_marker_robot* sns_msg_marker_robot_alloc( uint32_t _n );
  
  void sns_msg_marker_robot_dump( FILE* _out,
				  const struct sns_msg_marker_robot *_msg );


  
  /**
   * @struct sns_sq_info
   */
  struct sns_sq_info {
    
    /** Dimensions: a,b,c */
    double dim[3];
    
    /** Coefficients: e1 & e2 */
    double e[2];
    
    /** Translation & Rotation */
    double trans[3];
    double rot[3];

    /** Mesh info */
    char mesh_filename[255];
    bool mesh_generated;
  };
  
  struct sns_msg_tabletop_sq {
    
    struct sns_msg_header header;
    uint32_t n; /** Number of clusters */
    uint32_t i; /** Selected object to pick */
    /** Table info */
    double table_coeffs[4];
    char table_meshfile[255];
    /** Objects info */
    sns_sq_info u[1];
  };

  // Create sns_msg_tabletop_sq
  SNS_DEF_MSG_VAR( sns_msg_tabletop_sq, u );

  // Declarations
  struct sns_msg_tabletop_sq* sns_msg_tabletop_sq_alloc( uint32_t _n );
  
  void sns_msg_tabletop_sq_dump( FILE* _out,
				 const struct sns_msg_tabletop_sq *_msg );


  
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
   * @struct sns_msg_skel
   * @brief Define a Skeleton (with 9 joints)
   */
  struct sns_msg_skel {
    struct sns_msg_header header;
    // (x,y,z) coordinates of each of the 9 joints
    double x[9];
    double y[9];
    double z[9];
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


    /**
   * @struct sns_msg_depth_img
   * @brief Define a Depth image (XYZ)
   */
  struct sns_msg_depth_img {
    uint32_t width;
    uint32_t height;
    struct {
      float x;
      float y;
      float z;
    } u[1];
  };

  static inline size_t sns_msg_depth_img_size_tn( uint32_t _width, uint32_t _height ) {
    static const struct sns_msg_depth_img* msg;
    return sizeof( *msg ) + sizeof( msg->u[0] ) + sizeof( msg->u[0] )*_width*_height;
  }

  static inline size_t sns_msg_depth_img_size( const struct sns_msg_depth_img* _msg ) {
    return sns_msg_depth_img_size_tn( _msg->width, _msg->height );
  }

  struct sns_msg_depth_img* sns_msg_depth_img_alloc( uint32_t _width, uint32_t _height );
  void sns_msg_depth_img_dump( FILE* _out,
			       const struct sns_msg_depth_img* _msg );

 
  

#ifdef __cplusplus
}
#endif
