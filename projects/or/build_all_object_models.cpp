/**
 * @file build_all_object_models.cpp
<<<<<<< HEAD
 * @brief
 */

=======
 */
>>>>>>> 0f63159f09805fbb92ff2fbcca8d306432c92949
#include <string>
#include <sstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>

#include <obj_recog/object_recognition.h>
<<<<<<< HEAD

namespace bf = boost::filesystem;


// Functions
void help( char* argv0 );
inline void
getModelsInDirectory (bf::path & dir,
		      std::string & rel_path_so_far,
		      std::vector<std::string> & relative_paths );

/**
 * @function main
 */
int main ( int argc, char *argv[] ) {

  if (argc < 3) {
    help(argv[0]);
    return 1;
  }

  //-- Load parameters
  ObjectRecognitionParameters params;
  if( !params.loadParams( argv[2] ) ) {
    printf("DEBUG: Error loading parameters \n");
    return 1;
  }

  //-- Load directory
  std::string directory ( argv[1] );
  
  //-- Find all raw* files in input_directory
=======
#include <json/json.h>

namespace bf = boost::filesystem;

bool parse_RecogParams( ObjectRecognitionParameters &_params,
			std::string filename );
void print( ObjectRecognitionParameters _params );

inline void
getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
{
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (*itr))
    {
#if BOOST_FILESYSTEM_VERSION == 3
      std::string so_far = rel_path_so_far + itr->path ().filename ().string () + "/";
#else
      std::string so_far = rel_path_so_far + itr->path ().filename () + "/";
#endif
      bf::path curr_path = itr->path ();
      getModelsInDirectory (curr_path, so_far, relative_paths);
    }
    else
    {
      std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
      std::string file = itr->path ().filename ().string ();
#else
      std::string file = itr->path ().filename ();
#endif
      boost::split (strs, file, boost::is_any_of ("."));
      std::string extension = strs[strs.size () - 1];

      if((file.compare (0, 3, "raw") == 0) && extension == "pcd") {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string path = rel_path_so_far + itr->path ().filename ().string ();
#else
        std::string path = rel_path_so_far + itr->path ().filename ();
#endif
        relative_paths.push_back (path);
      }
    }
  }
}

int
main (int argc, char ** argv)
{
  if (argc < 3)
  {
    pcl::console::print_info ("Syntax is: %s input_directory <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("Note: The output's base filename must be specified without the .pcd extension\n");
    pcl::console::print_info ("      Four output files will be created with the following suffixes:\n");
    pcl::console::print_info ("        * '_points.pcd'\n");
    pcl::console::print_info ("        * '_keypoints.pcd'\n");
    pcl::console::print_info ("        * '_localdesc.pcd'\n");
    pcl::console::print_info ("        * '_globaldesc.pcd'\n");

    return (1);
  }

  ObjectRecognitionParameters params;
  


  std::string directory (argv[1]);
  //Find all raw* files in input_directory
>>>>>>> 0f63159f09805fbb92ff2fbcca8d306432c92949
  bf::path dir_path = directory;
  std::vector < std::string > files;
  std::string start = "";
  getModelsInDirectory (dir_path, start, files);

<<<<<<< HEAD
  printf("Number of files encountered : %d \n", files.size() );
  
  for(size_t i=0; i < files.size(); i++) {
    printf("Processing point %d \n", i);
    // Load input files
    std::string filename = directory;
    filename.append("/");
    filename.append(files[i]);
    
    PointCloudPtr input (new PointCloud);
    pcl::io::loadPCDFile (filename, *input);
    printf("Loaded %s (%lu points)\n", filename.c_str(), input->size ());
    
    std::cout << files[i] << std::endl;

    // Construct the object model
    ObjectRecognition obj_rec (params);
    ObjectModel model;
    obj_rec.constructObjectModel (input, model, true );

    // Get directory name
    std::vector < std::string > strs;
    boost::split (strs, files[i], boost::is_any_of ("/\\"));
    std::cout << "strs[0]: "<< strs[0] << " and strs[1]: "<< strs[1] << std::endl;
=======
  for(size_t i=0; i < files.size(); i++) {
    // Load input file

    std::string filename = directory;
    filename.append("/");
    filename.append(files[i]);
    PointCloudPtr input (new PointCloud);
    pcl::io::loadPCDFile (filename, *input);
    pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str(), input->size ());

    std::cout << files[i] << std::endl;
    // Construct the object model
    ObjectRecognition obj_rec (params);
    ObjectModel model;
    obj_rec.constructObjectModel (input, model);

    //get directory name
    std::vector < std::string > strs;
    boost::split (strs, files[i], boost::is_any_of ("/\\"));

>>>>>>> 0f63159f09805fbb92ff2fbcca8d306432c92949
    std::string id = strs[0];
    std::string raw_file = strs[1];

    strs.clear();
    boost::split (strs, raw_file, boost::is_any_of ("_"));

    std::stringstream base_filestream;
    base_filestream << directory << "/" << id << "/" << id << strs[1].substr(0,1);
    // Save the model files
    std::string base_filename (base_filestream.str()), output_filename;

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
  }

  return (0);
}


/**
<<<<<<< HEAD
 * @function help
 * @brief
 */
void help( char* argv0 ) {
  
  printf ("Syntax is: %s input_directory params.json outputname \n", argv0 );
  printf ("Note: The output's base filename must be specified without the .pcd extension\n");
  printf ("      Four output files will be created with the following suffixes:\n");
  printf ("        * '_points.pcd'\n");
  printf ("        * '_keypoints.pcd'\n");
  printf ("        * '_localdesc.pcd'\n");
  printf ("        * '_globaldesc.pcd'\n");

}


inline void
getModelsInDirectory (bf::path & dir,
		      std::string & rel_path_so_far,
		      std::vector<std::string> & relative_paths ) {
  bf::directory_iterator end_itr;

  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (*itr))
    {
#if BOOST_FILESYSTEM_VERSION == 3
      std::string so_far = rel_path_so_far + itr->path ().filename ().string () + "/";
#else
      std::string so_far = rel_path_so_far + itr->path ().filename () + "/";
#endif
      bf::path curr_path = itr->path ();
      getModelsInDirectory (curr_path, so_far, relative_paths);
    }
    else
    {
      std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
      std::string file = itr->path ().filename ().string ();
#else
      std::string file = itr->path ().filename ();
#endif
      boost::split (strs, file, boost::is_any_of ("."));
      std::string extension = strs[strs.size () - 1];
      // (file.compare (0, 3, "raw") == 0) &&
      if( extension == "pcd") {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string path = rel_path_so_far + itr->path ().filename ().string ();
#else
        std::string path = rel_path_so_far + itr->path ().filename ();
#endif
        relative_paths.push_back (path);
      }
    }
  }
=======
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
  
>>>>>>> 0f63159f09805fbb92ff2fbcca8d306432c92949
}
