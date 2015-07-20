#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/shapes.h>

int main( int argc, char* argv[] ) {

  //
  Eigen::Vector3f trans;
  Eigen::Quaternionf q;
  double a, b, c;
  std::string gOriginalFile;  

  int v;
  while( (v=getopt(argc,argv,"x:y:z:r:p:q:n:a:b:c:f:h")) != -1 ) {
  switch(v) {
  case 'f': { gOriginalFile.assign(optarg); } break;
  case 'x': { trans(0) = atof(optarg); } break;
  case 'y': { trans(1) = atof(optarg); } break;
  case 'z': { trans(2) = atof(optarg); } break;
  case 'r': { q.x() = atof(optarg); } break;
  case 'p': { q.y() = atof(optarg); } break;
  case 'q': { q.z() = atof(optarg); } break;
  case 'n': { q.w() = atof(optarg); } break;
  case 'a': { a = atof(optarg); } break;
  case 'b': { b = atof(optarg); } break;
  case 'c': { c = atof(optarg); } break;
  }
  }
  

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viz"));
  viewer->initCameraParameters ();
  viewer->addCoordinateSystem(0.2);
  // Add pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA>() );
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table( new pcl::PointCloud<pcl::PointXYZRGBA>() );
  pcl::io::loadPCDFile<pcl::PointXYZRGBA> (gOriginalFile, *cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("data_july_18/table_points.pcd", *table);
  viewer->addPointCloud( cloud, "cloud_original");
  viewer->addPointCloud( table, "table");
  // Add point
  pcl::PointXYZ cp;
  cp.x = trans(0); cp.y = trans(1); cp.z = trans(2);
  viewer->addSphere( cp, 0.005, "sphere" );
  // Add cube
  viewer->addCube( trans, q, 2*a,2*b,2*c);

  
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}
