/**
 * @file load_clouds.cpp
 * @brief
 */
#include "load_clouds.h"

/**
 * @function loadPoints
 * @brief Load points from filename_points.pcd
 */
PointCloudPtr loadPoints ( std::string filename ) {

  PointCloudPtr output (new PointCloud);
  filename.append ("_points.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

/**
 * @function loadSurfaceNormals
 * @brief Load normals from filename_normals.pcd
 */
SurfaceNormalsPtr loadSurfaceNormals(std::string filename) {

  SurfaceNormalsPtr output (new SurfaceNormals);
  filename.append ("_normals.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

/**
 * @function loadSurfaceNormals
 * @brief Load normals from filename_keypoints.pcd
 */
PointCloudPtr loadKeypoints( std::string filename ) {

  PointCloudPtr output (new PointCloud);
  filename.append ("_keypoints.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

/**
 * @function loadLocalDescriptors
 * @brief Load normals from filename_localdesc.pcd
 */
LocalDescriptorsPtr loadLocalDescriptors (std::string filename) {
  LocalDescriptorsPtr output (new LocalDescriptors);
  filename.append ("_localdesc.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

/**
 * @function loadGlobalDescriptors
 * @brief Load normals from filename_globaldesc.pcd
 */
GlobalDescriptorsPtr loadGlobalDescriptors ( std::string filename ) {
  GlobalDescriptorsPtr output (new GlobalDescriptors);
  filename.append ("_globaldesc.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

