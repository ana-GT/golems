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

/**
 * @function main
 */
int main (int argc, char *argv[] ) {

  if( argc != 3 ) {
    printf( " Syntax: %s pointcloud.pcd recog_params.json basename\n", argv[0] );
  }
  
  // Load input file
  PointCloudPtr input (new PointCloud);
  
  pcl::io::loadPCDFile (argv[1], *input);
  printf("\t * DEBUG: Loaded pointcloud %s with %d points \n", argv[1], input->size() );
  
  // Load object recognition parameters
  ObjectRecognitionParameters params;
  if( !params.loadParams( argv[2] ) ) {
    printf("\t * [BAD] Did not parse recognition parameters well \n");
    return -1;
  }
  
  // Construct the object model
  ObjectRecognition obj_rec (params);
  ObjectModel model;
  obj_rec.constructObjectModel (input, model);

  // Save the model files
  std::string base_filename (argv[3]), output_filename;

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
  
  return (0); 
}

