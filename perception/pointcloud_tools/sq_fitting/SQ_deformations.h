/**
 * @file SQ_deformations.h
 */
#pragma once

#include <pcl/io/pcd_io.h>

class SQ_deformations {
public:

  pcl::PointCloud<pcl::PointXYZ>::Ptr linear_tampering( const double &_a1,
							const double &_a2,
							const double &_a3,
							const double &_e1,
							const double &_e2,
							const double &_t );
  
protected:

};
