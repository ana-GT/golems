/**
 * @file marker_sendPose.cpp
 * @brief Detects marker 29 and send an ACH message with its pose
 * @author A. Huaman Q.
 */
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <Eigen/Core>

#include <aoi/utils/robotData.h>
#include <aoi/utils/globalStructs.h>
#include <dart/utils/urdf/DartLoader.h>
#include <motion_control/DualLimb_Interface.h>

#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/BodyNode.h>

#include <marker_detection/crichton_markerDetector.h>
#include <calibration/calibration_utils.h>
#include <global/crichton_global.h>

int mID = 29; // A prime number :0}

cv::Mat mRgbImg;
cv::Mat mDepthImg;
std::string mWindowName( "Marker Send Pose" );


ach_channel_t mMarkerPose_chan;
crichton_markerDetector mMd;


/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int, void* );
void openComm();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
 
  // Open Communication
  openComm();
  
  // Marker detector
  mMd.setMarkerSize(5);
  
  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 ); 
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }
  printf("\t * Opened device \n");

  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on

  printf("\t * RGB dimensions: (%f,%f), Depth dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ),
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ) );
  
  // Set control panel
  cv::namedWindow( mWindowName );
  // Set mouse callback 
  cv::setMouseCallback( mWindowName, onMouse, 0 );

  
  // Loop
  std::vector<calib_marker> markers;
      
  for(;;) {
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }

    capture.retrieve( mRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    capture.retrieve( mDepthImg, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    
    mMd.detect( mRgbImg );
    markers = mMd.getCalibMarkers();
    // Check if id was found
    int ind;
    for( int k = 0; k < markers.size(); ++k ) {
      if( mID == markers[k].id ) {
	ind = k;
	// If so, send the pose through the channel
	// Get position in Kinect coordinates
	cv::Point3f p; p = mDepthImg.at<cv::Point3f>( markers[k].py, markers[k].px );
	p.x = -p.x; // Remember to switch
	// Check if it is valid
	if( p.x == 0 && p.y == 0 && p.z == 0 ) { break; }	
	// If valid, send
	double pos[3];
	pos[0] = p.x; pos[1] = p.y; pos[2] = p.z;
	printf("Pos size: %d: [%f %f %f] \n", sizeof(pos), pos[0], pos[1], pos[2] );
	int r = ach_put( &mMarkerPose_chan,
			 pos,
			 sizeof(pos) );
	if( r != ACH_OK ) { printf("Something wrong sending the message \n"); }
	break;
      }
    }


    

    cv::imshow( mWindowName, mRgbImg );
    
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }
    aa_mem_region_local_release();
    
  } // end for
  
  return 0;
}


/**
 * @brief Check depth with a click
 */
static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  p = mDepthImg.at<cv::Point3f>(y,x);
  printf("Depth: %f %f %f \n", p.x, p.y, p.z); 
}

/**
 * @function openComm
 */
void openComm() {
  
  // Start sns
  sns_init();
  sns_start();

  // Open channels to send pose
  sns_chan_open( &mMarkerPose_chan, gMarker_pose_chan_name.c_str(), NULL );
  
  // Ready
  printf("\t * [SET_COMM] We should be ready to go \n");  
}


