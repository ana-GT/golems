/**
 * @file local_recognition.cpp
 */
#include <pcl/io/pcd_io.h>
#include <tabletop_segmentation/tabletop_segmentation.h>
#include "LocalRecognizer.h"

LocalRecognizer mLr;
std::string objects_input("/home/ana/Research/golems/projects/obj_recog/models_input.json");
pcl::PointCloud<PointType>::Ptr scene( new pcl::PointCloud<PointType>() );

int main( int argc, char* argv[] ) {

  if( argc != 2 ) {
    printf("Syntax: %s scene.pcd", argv[0] );
    return -1;
  }

  // Load scene
  pcl::io::loadPCDFile( argv[1], *scene );

  // Load models
  mLr.load( objects_input );
  mLr.prepareModels();
  mLr.printInfo();
  printf("\t ** Prepare models \n");


  // View keypoints
  //mLr.viewModels();

  // Load scene & set descriptors
  mLr.setScene( scene );

  // Matching
  mLr.matching();

  // Alignment
  mLr.correspondenceGrouping();

  // Absolute transform
  
  mLr.viewRecognition();

  return 0;
}
