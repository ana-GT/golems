/**
 * @file build_all_object_models.cpp
 * @brief
 */

#include <string>
#include <sstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>

#include <obj_recog/object_recognition.h>

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
  bf::path dir_path = directory;
  std::vector < std::string > files;
  std::string start = "";
  getModelsInDirectory (dir_path, start, files);

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
}
