
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/shapes.h>


#include <SQ_utils.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

typedef pcl::PointXYZRGBA PointT;

const int gnF = 5;
char* fx_names[gnF] = { "Radial", "Solina", "Ichim", "Chevalier", "F5"};

bool regularFlag = true;
std::string gGroupName;
std::string datapath("/home/ana/Research/golems/bin/data_july_18");
int fx = 0;

int main( int argc, char* argv[] ) {

  int v;
  while( (v=getopt(argc,argv,"n:r:f:h"))!= -1 ) {

    switch(v) {
    case 'r': {
      int c = atoi(optarg);
      if( c == 0 ) { regularFlag = true; }
      else { regularFlag = false; }
    } break;
    case 'n': {
      gGroupName.assign(optarg);
    } break;
    case 'h': {
      printf("Syntax: %s -n GROUP_NAME -r REGULAR_FLAG \n", argv[0]); return 0;
    } break;
    case 'f': {
      fx = atoi(optarg);
    } break;
    }
  }

  // Name of viewer
  char vizname[200];
  if( regularFlag ) {
    sprintf( vizname, "%s_%s_regular", gGroupName.c_str(), fx_names[fx] );
  } else {
    sprintf( vizname, "%s_%s_hier", gGroupName.c_str(), fx_names[fx] );
  }
  
  // Read group name
  char dataFile[200]; 
  sprintf( dataFile, "%s/%s/%s_data.txt",
	   datapath.c_str(),
	   gGroupName.c_str(),
	   gGroupName.c_str() );

  // Read data file to get number of objects 
  std::string line; int gT;
  std::ifstream input( dataFile, std::ifstream::in );
  std::getline(input, line); // gF
  std::getline(input, line); // gTableCoeff
  input >> gT;
  input.close();

  // Draw table
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (vizname));
  viewer->initCameraParameters ();
  viewer->addCoordinateSystem(0.2);

  // Load table
  pcl::PointCloud<PointT>::Ptr table( new pcl::PointCloud<PointT>() );
  char tablename[200]; sprintf(tablename,"%s/table_points.pcd", datapath.c_str() );
  pcl::io::loadPCDFile<PointT> (tablename, *table);
  viewer->addPointCloud( table, "table");

  
  // Read all objects
  for( int i = 0; i < gT; ++i ) {
    
    // Original
    pcl::PointCloud<PointT>::Ptr orig( new pcl::PointCloud<PointT>() );     
    char origname[200];
    sprintf(origname, "%s/%s/%s_%d.pcd", 
	    datapath.c_str(),
	    gGroupName.c_str(),
	    gGroupName.c_str(), i );
    // Load
    pcl::io::loadPCDFile<PointT> (origname, *orig);
    viewer->addPointCloud( orig, origname);

    // Approx
    pcl::PointCloud<pcl::PointXYZ>::Ptr app( new pcl::PointCloud<pcl::PointXYZ>() );     
    char appname[200];
    if( regularFlag ) {
      sprintf(appname, "%s/rn/%s_%d_%d.pcd", 
	    datapath.c_str(),
	    gGroupName.c_str(),
	    fx, i );
    } else {
      sprintf(appname, "%s/hn/%s_%d_%d.pcd", 
	      datapath.c_str(),
	      gGroupName.c_str(),
	      fx, i );
    }
    // Load
    pcl::io::loadPCDFile<pcl::PointXYZ> (appname, *app);
    viewer->addPointCloud( app, appname);    
  }

    // Visualize
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  
  
}


