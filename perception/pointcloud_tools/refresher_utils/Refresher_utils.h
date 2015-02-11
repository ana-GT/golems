/**
 * @file Refresher_utils.h
 */
#pragma once

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>

namespace dart { namespace dynamics { class Skeleton; } }


void downsample( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		 const double &_voxelSize,
		 pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud_downsampled );
bool pointcloudToMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
		       std::string _filename,
		       Eigen::Vector3i _color );
//bool SQToMesh( SQ_parameters _param );
bool saveMeshFile( pcl::PointCloud<pcl::PointXYZ> _points,
		   std::vector<pcl::Vertices> _faces,
		   std::string _filename,
		   Eigen::Vector3i _color );


void pointcloudToBB( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
		     double dim[3],
		     Eigen::Isometry3d &_Tf );
