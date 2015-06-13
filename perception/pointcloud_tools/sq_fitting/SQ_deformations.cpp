/**
 * @file SQ_deformations.cpp
 */

#include "SQ_deformations.h"
#include "SQ_utils.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr SQ_deformations::linear_tampering( const double &_a1,
								       const double &_a2,
								       const double &_a3,
								       const double &_e1,
								       const double &_e2,
								       const double &_t ) {

  // 0. Output
  pcl::PointCloud<pcl::PointXYZ>::Ptr output( new pcl::PointCloud<pcl::PointXYZ>() );

  // 1. Create the superellipsoid
  pcl::PointCloud<pcl::PointXYZ>::Ptr sq( new pcl::PointCloud<pcl::PointXYZ>() );
  sq = sampleSQ_uniform( _a1, _a2, _a3, _e1, _e2, 50 );

  // 2. Apply the deformation
  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  pcl::PointXYZ p;
  double a, b;

  for( it = sq->begin(); it != sq->end(); ++it ) {
    p = *it;
    p.y = ( 1 + (_t/_a3)*p.z )*p.y;
    p.x = ( 1 + (_t/_a3)*p.z )*p.x;
    output->points.push_back( p );
  }

  output->width = 1; output->height = output->points.size();
  return output;
}
