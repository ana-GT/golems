

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>


template<typename PointT>
void getBoundingBoxTable( const typename pcl::PointCloud<PointT>::Ptr _cloud,
			  double coeffs[4],
			  double dim[3],
			  double trans[3],
			  double rot[3] );

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr projectToPlane( typename pcl::PointCloud<PointT>::Ptr _cloud,
						      Eigen::VectorXd _planeCoeffs,
						      double &_dmin, double &_dmax );

template<typename PointT>
void getInfoFromProjectedCloud( typename pcl::PointCloud<PointT>::Ptr _cloud,
				double _voxel_size,
				Eigen::Vector3d &_center,
				Eigen::Vector3d &_ea,
				Eigen::Vector3d &_eb,
				double &_da, 
				double &_db );



/////////////////////////////////////////////////////
#include "perception/pointcloud_tools/tabletop_symmetry/impl/mindGapper_utils.hpp"
