/**
 * @file object_recognition.cpp
 * @brief Adapted from original source files from iccv2011 from PCL
 */
#include "object_recognition.h"

/**
 * @function populateDatabase
 * @brief Load filenames of object info stored (either pointclouds alone or with descriptors/keypoints)
 */
void ObjectRecognition::populateDatabase (const std::vector<std::string> & filenames) {
  
  size_t n = filenames.size ();
  models_.resize (n);
  descriptors_ = GlobalDescriptorsPtr (new GlobalDescriptors);

  for (size_t i = 0; i < n; ++i) {

    const std::string & filename = filenames[i];
    if (filename.compare (filename.size ()-4, 4, ".pcd") == 0) {
      // If filename ends pcd extension, load the points and process them
      PointCloudPtr raw_input (new PointCloud);
      pcl::io::loadPCDFile (filenames[i], *raw_input);
      
      constructObjectModel (raw_input, models_[i]);
    }
    else {
      // If the filename has no extension, load the pre-computed models
      models_[i].points = loadPointCloud<PointT> (filename, "_points.pcd");
      models_[i].keypoints = loadPointCloud<PointT> (filename, "_keypoints.pcd");
      models_[i].local_descriptors = loadPointCloud<LocalDescriptorT> (filename, "_localdesc.pcd");
      models_[i].global_descriptor = loadPointCloud<GlobalDescriptorT> (filename, "_globaldesc.pcd");      
    }
    // For every pointcloud entered, add descriptor
    *descriptors_ += *(models_[i].global_descriptor);
  }

  kdtree_ = pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr (new pcl::KdTreeFLANN<GlobalDescriptorT>);
  kdtree_->setInputCloud (descriptors_);
} 

/**
 * @function recognizeObject
 * @brief Perform best matching of the object global descriptor with the database
 */
const ObjectModel & ObjectRecognition::recognizeObject (const PointCloudPtr & query_cloud) {

  ObjectModel query_object;

  constructObjectModel (query_cloud, query_object);
  const GlobalDescriptorT & query_descriptor = query_object.global_descriptor->points[0];
  
  std::vector<int> nn_index (1);
  std::vector<float> nn_sqr_distance (1);
  kdtree_->nearestKSearch (query_descriptor, 1, nn_index, nn_sqr_distance);
  const int & best_match = nn_index[0];
  
  return (models_[best_match]);
}

/**
 * @function recognizeAndAlignPoints
 * @brief Aligns the best match with the query object pointcloud
 */
PointCloudPtr ObjectRecognition::recognizeAndAlignPoints(const PointCloudPtr & query_cloud) {

  ObjectModel query_object;
  constructObjectModel (query_cloud, query_object);
  const GlobalDescriptorT & query_descriptor = query_object.global_descriptor->points[0];
  
  std::vector<int> nn_index (1);
  std::vector<float> nn_sqr_distance (1);
  kdtree_->nearestKSearch (query_descriptor, 1, nn_index, nn_sqr_distance);
  const int & best_match = nn_index[0];
  
  PointCloudPtr output = alignModelPoints (models_[best_match], query_object, params_);
  return (output);
}


/**
 * Construct an object model by filtering, segmenting, and estimating feature descriptors 
 */
void ObjectRecognition::constructObjectModel( const PointCloudPtr & points,
					      ObjectModel & output ) const {
  printf("About to filter and segment \n");
  output.points = applyFiltersAndSegment (points, params_);
  
  SurfaceNormalsPtr normals;
  printf("Estimate features \n");
  estimateFeatures (output.points, params_, normals, output.keypoints, 
		    output.local_descriptors, output.global_descriptor);
}


/** 
 * @function applyFiltersAndSegment
 * @brief Apply a series of filters (threshold depth, downsample, and remove outliers) 
 */
PointCloudPtr ObjectRecognition::applyFiltersAndSegment (const PointCloudPtr & input,
							 const ObjectRecognitionParameters & params) const {
  PointCloudPtr cloud;
  printf("Trheshold \n");
  cloud = thresholdDepth (input, params.min_depth, params.max_depth);
  printf("Downsample \n");
  cloud = downsample (cloud, params.downsample_leaf_size);
  printf("Remove outliers \n");
  cloud = removeOutliers (cloud, params.outlier_rejection_radius, params.outlier_rejection_min_neighbors);
  
  printf("Substract plane \n");
  cloud = findAndSubtractPlane (cloud, params.plane_inlier_distance_threshold, params.max_ransac_iterations);
  std::vector<pcl::PointIndices> cluster_indices;
  printf("Cluster objects \n");
  clusterObjects (cloud, params.cluster_tolerance, params.min_cluster_size, 
		  params.max_cluster_size, cluster_indices);
  printf("Num of clusters: %d \n", cluster_indices.size());
  PointCloudPtr largest_cluster (new PointCloud);
  printf("Copy cluster \n");
  pcl::copyPointCloud (*cloud, cluster_indices[0], *largest_cluster);
  
  return (largest_cluster);
}

/**
 * @function
 * @brief Estimate surface normals, keypoints, and local/global feature descriptors 
 */
void ObjectRecognition::estimateFeatures ( const PointCloudPtr & points,
					   const ObjectRecognitionParameters & params,
					   SurfaceNormalsPtr & normals_out,
					   PointCloudPtr & keypoints_out, 
					   LocalDescriptorsPtr & local_descriptors_out,
					   GlobalDescriptorsPtr & global_descriptor_out ) const {
  printf("Estimate normals \n");
  normals_out = estimateSurfaceNormals (points, params.surface_normal_radius);
  printf("Detect keypoints \n");
  keypoints_out = detectKeypoints( points, normals_out,
				   params.keypoints_min_scale,
				   params.keypoints_nr_octaves,
				   params.keypoints_nr_scales_per_octave,
				   params.keypoints_min_contrast );
  printf("Compute local descriptors \n");
  local_descriptors_out = computeLocalDescriptors (points, normals_out, keypoints_out, 
						   params.local_descriptor_radius);
  printf("Compute global descriptors \n");
  global_descriptor_out = computeGlobalDescriptor (points, normals_out);
}

/**
 * @function
 * @brief  Align the points in the source model to the points in the target model 
 */
PointCloudPtr ObjectRecognition::alignModelPoints( const ObjectModel & source,
						   const ObjectModel & target, 
						   const ObjectRecognitionParameters & params) const {
  Eigen::Matrix4f tform; 
  tform = computeInitialAlignment (source.keypoints, source.local_descriptors,
				   target.keypoints, target.local_descriptors,
				   params.initial_alignment_min_sample_distance,
				   params.initial_alignment_max_correspondence_distance, 
				   params.initial_alignment_nr_iterations);
  
  tform = refineAlignment (source.points, target.points, tform, 
			   params.icp_max_correspondence_distance, params.icp_outlier_rejection_threshold, 
			   params.icp_transformation_epsilon, params.icp_max_iterations);
  
  PointCloudPtr output (new PointCloud);
  pcl::transformPointCloud (*(source.points), *output, tform);
  
  return (output);
}  

