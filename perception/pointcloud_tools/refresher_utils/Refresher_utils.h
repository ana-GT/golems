/**
 * @file Refresher_utils.h
 */
#pragma once

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include <pcl/filters/voxel_grid.h>

template<typename T>
void downsample( const boost::shared_ptr<const pcl::PointCloud<T> > &_cloud,
		 const double &_voxelSize,
		 pcl::PointCloud<T> &_cloud_downsampled ) {

  // Create the filtering object
  pcl::VoxelGrid<T> downsampler;
  // Set input cloud
  downsampler.setInputCloud( _cloud );
  // Set size of voxel
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  // Downsample
  downsampler.filter( _cloud_downsampled );  
}

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
