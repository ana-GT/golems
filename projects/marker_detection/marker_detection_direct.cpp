/**
 * @file marker_detection_direct.cpp
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sns.h>
#include <stdint.h>
#include <ach.h>

#include <msgs/perception_msgs.h>

#include "crichton_markerDetector.h"

cv::Mat rgbImg, pclMap;
std::string mWindowName( "Crichton View" );

struct timespec t_now;
struct timespec t_timeout;

crichton_markerDetector mMd;

static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  p = pclMap.at<cv::Point3f>(y,x);
  printf("Depth: %f %f %f \n", p.x, p.y, p.z);
 
}



/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 ); 
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }
  printf("\t * Opened device \n");
  
  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 1.0 );
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on
  
  printf("\t * Common Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_PROP_FRAME_HEIGHT ) );
  
  printf("\t * Only Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ) );

  printf("\t * Only Depth dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ) );

  
  // Set control panel
  int value;
  cv::namedWindow( mWindowName );
  // Set mouse callback 
  cv::setMouseCallback( mWindowName, onMouse, 0 );

  // Setup
  mMd.setMarkerSize(5);


  // Loop
  for(;;) {

    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }
    
    capture.retrieve( rgbImg, cv::CAP_OPENNI_BGR_IMAGE );

    capture.retrieve( pclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );

  
    // Detect markers
    if( mMd.detect(rgbImg) ) {	      
    }
          cv::imshow( mWindowName, rgbImg );

    
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * PRESSED 'q'. Finishing the program \n");
      return 1;	
    }

  } // end for

  

  
    
}
