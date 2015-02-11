/**
 * @file object_recognition.cpp
 * @brief Adapted from original source files from iccv2011 from PCL
 */
#include "object_recognition.h"

/**
 * @function loadParams
 */
bool ObjectRecognitionParameters::loadParams( std::string _filename ) {
  
  Json::Value root;
  Json::Reader reader;
  
  std::ifstream obj_string( _filename.c_str(),
			    std::ifstream::binary );
  
  bool parsingSuccessful = reader.parse( obj_string, root );
  if( !parsingSuccessful ) {
    std::cout << "OH CRAP! Failed to parse file: \n "<< reader.getFormattedErrorMessages()<<std::endl;
    return false;
  }
  
  Json::Value filter_params = root["filter_parameters"];
  this->min_depth = filter_params.get("min_depth", 0.0).asFloat();
  this->max_depth = filter_params.get("max_depth", 0.0).asFloat();
  this->downsample_leaf_size = filter_params.get("downsample_leaf_size", 0.0).asFloat();
  this->outlier_rejection_radius = filter_params.get("outlier_rejection_radius", 0.0).asFloat();
  this->outlier_rejection_min_neighbors = filter_params.get("outlier_rejection_min_neighbors", 0).asInt();

  Json::Value segmentation_parameters = root["segmentation_parameters"];
  this->plane_inlier_distance_threshold = segmentation_parameters.get("plane_inlier_distance_threshold",0.1).asFloat();
  this->max_ransac_iterations = segmentation_parameters.get("max_ransac_iterations",10).asInt();
  this->cluster_tolerance = segmentation_parameters.get("cluster_tolerance",0.5).asFloat();
  this->min_cluster_size = segmentation_parameters.get("min_cluster_size",10).asInt();
  this->max_cluster_size = segmentation_parameters.get("max_cluster_size",100).asInt();


  Json::Value feature_estimation_parameters = root["feature_estimation_parameters"];
  this->surface_normal_radius = feature_estimation_parameters.get("surface_normal_radius", 0.1).asFloat();
  this->keypoints_min_scale = feature_estimation_parameters.get("keypoints_min_scale",0.1 ).asFloat();
  this->keypoints_nr_octaves = feature_estimation_parameters.get("keypoints_nr_octaves", 8 ).asFloat();
  this->keypoints_nr_scales_per_octave = feature_estimation_parameters.get("keypoints_nr_scales_per_octave", 1 ).asFloat();
  this->keypoints_min_contrast = feature_estimation_parameters.get("keypoints_min_contrast", 1 ).asFloat();
  this->local_descriptor_radius = feature_estimation_parameters.get("local_descriptor_radius", 2 ).asFloat();
  
  Json::Value registration_params = root["registration_parameters"];
  this->initial_alignment_min_sample_distance = registration_params.get("initial_alignment_min_sample_distance", 1 ).asFloat();
  this->initial_alignment_max_correspondence_distance = registration_params.get( "initial_alignment_max_correspondence_distance", 1 ).asFloat();
  this->initial_alignment_nr_iterations = registration_params.get( "initial_alignment_nr_iterations", 1 ).asInt();
  this->icp_max_correspondence_distance = registration_params.get( "icp_max_correspondence_distance", 1 ).asFloat();
  this->icp_outlier_rejection_threshold = registration_params.get( "icp_outlier_rejection_threshold", 1 ).asFloat();
  this->icp_transformation_epsilon = registration_params.get( "icp_transformation_epsilon", 1 ).asFloat();
  this->icp_max_iterations = registration_params.get( "icp_max_iterations", 1 ).asInt();
    
  return true;
  
}


/**
 * @function Operator <<
 * @brief
 */
std::ostream& operator << ( std::ostream& os,
			   const ObjectRecognitionParameters &_params ) {

  os << "**************************" << std::endl;
  os << "FILTER PARAMETERS " << std::endl;
  os << "Min depth: "<< _params.min_depth <<std::endl;
  os << "Max depth: " << _params.max_depth <<std::endl;
  os << "Downsample_leaf_size: " << _params.downsample_leaf_size <<std::endl;
  os << "Outlier_rejection_radius: " << _params.outlier_rejection_radius<<std::endl;
  os << "Outlier_rejection_min_neighbors: " << _params.outlier_rejection_min_neighbors <<std::endl;

  os <<"**************************"<<std::endl;
  os <<"SEGMENTATION PARAMETERS"<<std::endl;
  os << "plane_inlier_distance_threshold: "<< _params.plane_inlier_distance_threshold <<std::endl;
  os << "max_ransac_iterations: "<< _params.max_ransac_iterations <<std::endl;
  os << "cluster_tolerance: "<< _params.cluster_tolerance <<std::endl;
  os << "min_cluster_size: "<< _params.min_cluster_size <<std::endl;
  os << "max_cluster_size: "<< _params.max_cluster_size <<std::endl;

  os <<"**************************"<<std::endl;
  os <<"FEATURE ESTIMATION PARAMETERS"<<std::endl;
  os <<"surface_normal_radius: " << _params.surface_normal_radius <<std::endl;
  os <<"keypoints_min_scale: " << _params.keypoints_min_scale <<std::endl;
  os <<"keypoints_nr_octaves: " << _params.keypoints_nr_octaves <<std::endl;
  os <<"keypoints_nr_scales_per_octave: " << _params.keypoints_nr_scales_per_octave <<std::endl;
  os <<"keypoints_min_contrast: " << _params.keypoints_min_contrast <<std::endl;
  os <<"local_descriptor_radius:" << _params.local_descriptor_radius <<std::endl;

  os <<"**************************"<<std::endl;
  os <<" REGISTRATION PARAMETERS"<<std::endl;
  os <<"initial_alignment_min_sample_distance: " << _params.initial_alignment_min_sample_distance <<std::endl;
  os << "initial_alignment_max_correspondence_distance: " << _params.initial_alignment_max_correspondence_distance <<std::endl;
  os << "initial_alignment_nr_iterations: " << _params.initial_alignment_nr_iterations <<std::endl;
  os << "icp_max_correspondence_distance: " << _params.icp_max_correspondence_distance <<std::endl;
  os << "icp_outlier_rejection_threshold: " << _params.icp_outlier_rejection_threshold <<std::endl;
  os << "icp_transformation_epsilon: " << _params.icp_transformation_epsilon <<std::endl;
  os << "icp_max_iterations: "<< _params.icp_max_iterations <<std::endl;

}

/**
 * @function populateDatabase
 */
bool ObjectRecognition::populateDatabase( std::string _fileToParse ) {

  Json::Value root;
  Json::Reader reader;
  
  std::ifstream obj_string( _fileToParse.c_str(),
			    std::ifstream::binary );
  
  bool parsingSuccessful = reader.parse( obj_string, root );
  if( !parsingSuccessful ) {
    std::cout << "OH CRAP! Failed to parse file: \n "<< reader.getFormattedErrorMessages()<<std::endl;
    return false;
  }
  
  std::string objectName;
  std::vector<std::string> filenames;
    
  objectName = root.get("name", "noname").asString();
  Json::Value files = root["files"];
  for( int i = 0; i < files.size(); ++i ) {
    filenames.push_back( files[i].asString() );
  }
  
  
  printf("DEBUG: Object name: %s \n", objectName.c_str() );
  printf("DEBUG: Loading %d files \n", filenames.size() );

  //populateDatabase( filenames );  
}

/**
 * @function populateDatabase
 * @brief Load filenames of object info stored (either pointclouds alone or with descriptors/keypoints)
 */
bool ObjectRecognition::populateDatabase (const std::vector<std::string> & filenames) {
  
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

  return true;
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
					      ObjectModel & output,
					      bool _alreadySegmented ) const {

  if( !_alreadySegmented ) {
    printf("Apply filter and segment \n");
    output.points = applyFiltersAndSegment (points, params_);
  } else {
    output.points = points;
  }

  printf("Estimate features. Points have a size of : %d \n", output.points->points.size() );
  SurfaceNormalsPtr normals;
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
  printf("Size of normals: %d \n", normals_out->points.size() );
/*
 printf("Detect keypoints \n");
  keypoints_out = detectKeypoints( points, normals_out,
				   params.keypoints_min_scale,
				   params.keypoints_nr_octaves,
				   params.keypoints_nr_scales_per_octave,
				   params.keypoints_min_contrast );
 printf("Size of keypoints: %d \n", keypoints_out->points.size() );
  printf("Get local descriptors \n");
  local_descriptors_out = computeLocalDescriptors (points, normals_out, keypoints_out, 
						   params.local_descriptor_radius);
*/
  printf("Get global descriptors \n");
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

