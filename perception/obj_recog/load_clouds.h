#pragma once

#include "typedefs.h"

#include <pcl/io/pcd_io.h>
  
/**
 * @function loadPointCloud.cpp
 * @brief Load "filename.suffix" pointcloud
 * @todo REMEMBER template functions always go in headers! (so, don't move to a .cpp file)
 */  
template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT> >
loadPointCloud (std::string filename, std::string suffix) {

  boost::shared_ptr<pcl::PointCloud<PointT> > output (new pcl::PointCloud<PointT>);
  filename.append (suffix);
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

PointCloudPtr loadPoints (std::string filename);

SurfaceNormalsPtr loadSurfaceNormals(std::string filename);

PointCloudPtr loadKeypoints (std::string filename);

LocalDescriptorsPtr loadLocalDescriptors (std::string filename);

GlobalDescriptorsPtr loadGlobalDescriptors (std::string filename);
