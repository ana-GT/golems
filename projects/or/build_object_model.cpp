/**
 * @file build_object_model.cpp
 */

#include <string>
#include <sstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <obj_recog/object_recognition.h>

#include <json/json.h>

bool parse_RecogParams( ObjectRecognitionParameters &_params,
			std::string filename );
void print( ObjectRecognitionParameters _params );

/**
 * @function main
 */
int main (int argc, char *argv[] ) {

  if( argc != 3 ) {
    printf( " Syntax: %s  recog_params.json pointcloud.pcd \n", argv[0] );
  }
  
  // Load input file
  PointCloudPtr input (new PointCloud);
  /*
  pcl::io::loadPCDFile (argv[2], *input);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], input->size ());    
  */
  // Load object recognition parameters
  ObjectRecognitionParameters params;

  //Parse filter parameters
  if( !parse_RecogParams( params, argv[1] ) ) {
    printf("Did not parse recognition parameters well \n");
    return -1;
  } else {
    printf("Parsed recognition parameters fine \n");
  }

  // Construct the object model
  /*
  ObjectRecognition obj_rec (params);
  ObjectModel model;
  obj_rec.constructObjectModel (input, model);

  // Save the model files
  std::string base_filename (argv[2]), output_filename;

  output_filename = base_filename;
  output_filename.append ("_points.pcd");
  pcl::io::savePCDFile (output_filename, *(model.points));
  pcl::console::print_info ("Saved points as %s\n", output_filename.c_str ());

  output_filename = base_filename;
  output_filename.append ("_keypoints.pcd");
  pcl::io::savePCDFile (output_filename, *(model.keypoints));
  pcl::console::print_info ("Saved keypoints as %s\n", output_filename.c_str ());
  
  output_filename = base_filename;
  output_filename.append ("_localdesc.pcd");
  pcl::io::savePCDFile (output_filename, *(model.local_descriptors));
  pcl::console::print_info ("Saved local descriptors as %s\n", output_filename.c_str ());
  
  output_filename = base_filename;
  output_filename.append ("_globaldesc.pcd");
  pcl::io::savePCDFile (output_filename, *(model.global_descriptor));
  pcl::console::print_info ("Saved global descriptor as %s\n", output_filename.c_str ());
  */
  return (0); 
}

/**
 * @function parse_RecogParams
 */
bool parse_RecogParams( ObjectRecognitionParameters &_params,
			std::string filename ) {

  Json::Value root;
  Json::Reader reader;

  std::ifstream obj_string( filename.c_str(),
			    std::ifstream::binary );

  bool parsingSuccessful = reader.parse( obj_string, root );
  if( !parsingSuccessful ) {
    std::cout << "OH CRAP! Failed to parse file: \n "<< reader.getFormattedErrorMessages()<<std::endl;
    return false;
  }

  Json::Value filter_params = root["filter_parameters"];
  _params.min_depth = filter_params.get("min_depth", 0.0).asFloat();
  _params.max_depth = filter_params.get("max_depth", 0.0).asFloat();
  _params.downsample_leaf_size = filter_params.get("downsample_leaf_size", 0.0).asFloat();
  _params.outlier_rejection_radius = filter_params.get("outlier_rejection_radius", 0.0).asFloat();
  _params.outlier_rejection_min_neighbors = filter_params.get("outlier_rejection_min_neighbors", 0).asInt();

  Json::Value segmentation_parameters = root["segmentation_parameters"];
  _params.plane_inlier_distance_threshold = segmentation_parameters.get("plane_inlier_distance_threshold",0.1).asFloat();
  _params.max_ransac_iterations = segmentation_parameters.get("max_ransac_iterations",10).asInt();
  _params.cluster_tolerance = segmentation_parameters.get("cluster_tolerance",0.5).asFloat();
  _params.min_cluster_size = segmentation_parameters.get("min_cluster_size",10).asInt();
  _params.max_cluster_size = segmentation_parameters.get("max_cluster_size",100).asInt();


  Json::Value feature_estimation_parameters = root["feature_estimation_parameters"];
  _params.surface_normal_radius = feature_estimation_parameters.get("surface_normal_radius", 0.1).asFloat();
  _params.keypoints_min_scale = feature_estimation_parameters.get("keypoints_min_scale",0.1 ).asFloat();
  _params.keypoints_nr_octaves = feature_estimation_parameters.get("keypoints_nr_octaves", 8 ).asFloat();
  _params.keypoints_nr_scales_per_octave = feature_estimation_parameters.get("keypoints_nr_scales_per_octave", 1 ).asFloat();
  _params.keypoints_min_contrast = feature_estimation_parameters.get("keypoints_min_contrast", 1 ).asFloat();
  _params.local_descriptor_radius = feature_estimation_parameters.get("local_descriptor_radius", 2 ).asFloat();
  
  Json::Value registration_params = root["registration_parameters"];
  _params.initial_alignment_min_sample_distance = registration_params.get("initial_alignment_min_sample_distance", 1 ).asFloat();
  _params.initial_alignment_max_correspondence_distance = registration_params.get( "initial_alignment_max_correspondence_distance", 1 ).asFloat();
  _params.initial_alignment_nr_iterations = registration_params.get( "initial_alignment_nr_iterations", 1 ).asInt();
  _params.icp_max_correspondence_distance = registration_params.get( "icp_max_correspondence_distance", 1 ).asFloat();
  _params.icp_outlier_rejection_threshold = registration_params.get( "icp_outlier_rejection_threshold", 1 ).asFloat();
  _params.icp_transformation_epsilon = registration_params.get( "icp_transformation_epsilon", 1 ).asFloat();
  _params.icp_max_iterations = registration_params.get( "icp_max_iterations", 1 ).asInt();

     
  print( _params );
    
  return true;
}

/**
 * @function print
 * @brief
 */
void print( ObjectRecognitionParameters _params ) {

  printf("**************************\n");
  printf("FILTER PARAMETERS \n");
  printf( "Min depth: %f \n", _params.min_depth );
  printf( "Max depth: %f \n", _params.max_depth );
  printf( "Downsample_leaf_size: %f \n", _params.downsample_leaf_size );
  printf( "Outlier_rejection_radius: %f \n", _params.outlier_rejection_radius);
  printf( "Outlier_rejection_min_neighbors: %d \n", _params.outlier_rejection_min_neighbors );

  printf("**************************\n");
  printf("SEGMENTATION PARAMETERS \n");
  printf( "plane_inlier_distance_threshold: %f \n", _params.plane_inlier_distance_threshold );
  printf( "max_ransac_iterations: %d \n", _params.max_ransac_iterations );
  printf( "cluster_tolerance: %f \n", _params.cluster_tolerance );
  printf( "min_cluster_size: %d \n", _params.min_cluster_size );
  printf( "max_cluster_size: %d \n", _params.max_cluster_size );

  printf("**************************\n");
  printf("FEATURE ESTIMATION PARAMETERS \n");
  printf("surface_normal_radius: %f \n", _params.surface_normal_radius );
  printf("keypoints_min_scale: %f \n", _params.keypoints_min_scale );
  printf("keypoints_nr_octaves: %f \n", _params.keypoints_nr_octaves );
  printf("keypoints_nr_scales_per_octave: %f \n", _params.keypoints_nr_scales_per_octave );
  printf("keypoints_min_contrast: %f \n", _params.keypoints_min_contrast );
  printf("local_descriptor_radius: %f \n", _params.local_descriptor_radius );

  printf("**************************\n");
  printf(" REGISTRATION PARAMETERS \n");
  printf("initial_alignment_min_sample_distance: %f \n", _params.initial_alignment_min_sample_distance );
  printf( "initial_alignment_max_correspondence_distance: %f \n", _params.initial_alignment_max_correspondence_distance );
  printf( "initial_alignment_nr_iterations: %d \n", _params.initial_alignment_nr_iterations );
  printf( "icp_max_correspondence_distance: %f \n", _params.icp_max_correspondence_distance );
  printf( "icp_outlier_rejection_threshold: %f \n", _params.icp_outlier_rejection_threshold );
  printf( "icp_transformation_epsilon: %f \n", _params.icp_transformation_epsilon );
  printf( "icp_max_iterations: %d ", _params.icp_max_iterations );
  
}
