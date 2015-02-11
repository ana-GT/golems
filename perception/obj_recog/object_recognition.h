/**
 * @file object_recognition.h
 */

#pragma once


#include "typedefs.h"
#include "load_clouds.h"
#include "filters.h"
#include "segmentation.h"
#include "feature_estimation.h"
#include "registration.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <json/json.h>

/**
 * @structure ObjectRecognitionParameters
 */
struct ObjectRecognitionParameters {
  // Filter parameters
  float min_depth;
  float max_depth;
  float downsample_leaf_size;
  float outlier_rejection_radius;
  int outlier_rejection_min_neighbors;

  // Segmentation parameters
  float plane_inlier_distance_threshold;
  int max_ransac_iterations;
  float cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;

  // Feature estimation parameters
  float surface_normal_radius;
  float keypoints_min_scale;
  float keypoints_nr_octaves;
  float keypoints_nr_scales_per_octave;
  float keypoints_min_contrast;
  float local_descriptor_radius;

  // Registration parameters
  float initial_alignment_min_sample_distance;
  float initial_alignment_max_correspondence_distance;
  int initial_alignment_nr_iterations;
  float icp_max_correspondence_distance;
  float icp_outlier_rejection_threshold;
  float icp_transformation_epsilon;
  int icp_max_iterations;

  bool loadParams( std::string _filename );
  friend std::ostream& operator<<( std::ostream& os,
				   const ObjectRecognitionParameters& _params );
};

/**
 * @structure ObjectModel
 */
struct ObjectModel
{
  PointCloudPtr points;
  PointCloudPtr keypoints;
  LocalDescriptorsPtr local_descriptors;
  GlobalDescriptorsPtr global_descriptor;
};

/**
 * @class ObjectRecognition
 */
class ObjectRecognition {
 public:
  ObjectRecognition (const ObjectRecognitionParameters & params) : params_ (params)
  {}
  
  bool populateDatabase( std::string _fileToParse );
  bool populateDatabase (const std::vector<std::string> & filenames);
  const ObjectModel& recognizeObject (const PointCloudPtr & query_cloud);
  PointCloudPtr recognizeAndAlignPoints(const PointCloudPtr & query_cloud);
  void constructObjectModel (const PointCloudPtr & points,
			     ObjectModel & output,
			     bool _alreadySegmented = false ) const;
  
 protected: 
  
  PointCloudPtr applyFiltersAndSegment (const PointCloudPtr & input,
					const ObjectRecognitionParameters & params) const;
  
  void estimateFeatures (const PointCloudPtr & points,
			 const ObjectRecognitionParameters & params,
			 SurfaceNormalsPtr & normals_out,
			 PointCloudPtr & keypoints_out, 
			 LocalDescriptorsPtr & local_descriptors_out,
			 GlobalDescriptorsPtr & global_descriptor_out) const;
  
  PointCloudPtr alignModelPoints (const ObjectModel & source,
				  const ObjectModel & target, 
				  const ObjectRecognitionParameters & params) const;
  
  /** Members */
  ObjectRecognitionParameters params_;
  std::vector<ObjectModel> models_;
  GlobalDescriptorsPtr descriptors_;
  pcl::KdTreeFLANN<GlobalDescriptorT>::Ptr kdtree_;
};
