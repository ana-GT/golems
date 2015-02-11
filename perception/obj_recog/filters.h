/**
 * @file filters.h
 */

#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "typedefs.h"

/* Use a PassThrough filter to remove points with depth values that are too large or too small */
PointCloudPtr thresholdDepth (const PointCloudPtr & input, float min_depth, float max_depth);

/* Use a VoxelGrid filter to reduce the number of points */
PointCloudPtr downsample (const PointCloudPtr & input, float leaf_size);

/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
PointCloudPtr removeOutliers (const PointCloudPtr & input, float radius, int min_neighbors );

/* Apply a series of filters (threshold depth, downsample, and remove outliers) */
PointCloudPtr applyFilters ( const PointCloudPtr & input,
			     float min_depth,
			     float max_depth,
			     float leaf_size,
			     float radius, 
			     float min_neighbors );
