/**
 * @file registration.h
 */
#pragma once

#include "typedefs.h"

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

Eigen::Matrix4f computeInitialAlignment( const PointCloudPtr & source_points,
					 const LocalDescriptorsPtr & source_descriptors,
					 const PointCloudPtr & target_points,
					 const LocalDescriptorsPtr & target_descriptors,
					 float min_sample_distance,
					 float max_correspondence_distance,
					 int nr_iterations );
Eigen::Matrix4f refineAlignment( const PointCloudPtr & source_points,
				 const PointCloudPtr & target_points, 
				 const Eigen::Matrix4f &initial_alignment,
				 float max_correspondence_distance,
				 float outlier_rejection_threshold,
				 float transformation_epsilon,
				 float max_iterations );
