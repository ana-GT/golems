/**
 * @file feature_estimation.h
 */

#pragma once

#include "typedefs.h"

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>

/** 
 * @structure ObjectFeatures
 * @brief A simple structure for storing all of a cloud's features 
 */
struct ObjectFeatures {
  PointCloudPtr points;
  SurfaceNormalsPtr normals;
  PointCloudPtr keypoints;
  LocalDescriptorsPtr local_descriptors;
  GlobalDescriptorsPtr global_descriptor;
};



SurfaceNormalsPtr estimateSurfaceNormals( const PointCloudPtr & input,
					  float radius );

PointCloudPtr detectKeypoints ( const PointCloudPtr & points,
				const SurfaceNormalsPtr & normals,
				float min_scale,
				int nr_octaves,
				int nr_scales_per_octave,
				float min_contrast );

LocalDescriptorsPtr computeLocalDescriptors (const PointCloudPtr & points,
					     const SurfaceNormalsPtr & normals, 
					     const PointCloudPtr & keypoints,
					     float feature_radius);

GlobalDescriptorsPtr computeGlobalDescriptor (const PointCloudPtr & points,
					      const SurfaceNormalsPtr & normals);

ObjectFeatures computeFeatures (const PointCloudPtr & input );

