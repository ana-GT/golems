/**
 * @file global_recognition.cpp
 */
#include <pcl/io/pcd_io.h>
#include <tabletop_segmentation/tabletop_segmentation.h>
#include "GlobalRecognizer.h"

GlobalRecognizer mGr;
std::string objects_input("/home/ana/Research/golems/projects/obj_recog/models_input.json");

int main( int argc, char* argv[] ) {

  if( argc != 2 ) {
    printf("Syntax: %s scene.pcd", argv[0] );
    return -1;
  }

  // Load models
  mGr.load( objects_input );
  
  // Load scene
  
  // Segmentation
  TabletopSegmentor<pcl::PointXYZRGBA> tts;
  //tts.processCloud( cloud );
  
  
  // Description
  // Matching
  // Alignment

}
