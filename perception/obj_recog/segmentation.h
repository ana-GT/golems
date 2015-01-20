/**
 * @file segmentation.h
 * @brief
 */

#pragma once

#include "typedefs.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>


pcl::ModelCoefficients::Ptr fitPlane( const PointCloudPtr & input,
				      float distance_threshold,
				      float max_iterations );

PointCloudPtr findAndSubtractPlane( const PointCloudPtr & input,
				    float distance_threshold,
				    float max_iterations );

void clusterObjects (const PointCloudPtr & input, 
		     float cluster_tolerance,
		     int min_cluster_size, int max_cluster_size,
		     std::vector<pcl::PointIndices> & cluster_indices_out);
