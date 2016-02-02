/**
 * @file crichton_eye_v0
 * @brief Sends a message with the position in 3D of the point captured
 * @author A. Huaman Q.
 * @date 2015/01/02
 */
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <ach.h>
#include <sns.h>

#include "global/crichton_global.h"

cv::Mat rgbImg;
cv::Mat pclMap;
std::string mWindowName( "Crichton Eye v0" );

ach_channel_t mPoint_chan;
double mP[3];

/***********************/
/** FUNCTIONS          */
/***********************/
void startComm( int state, void* userdata );
void sendPoint( int state, void* userdata );
static void onMouse( int event, int x, int y, int, void* ); 

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 ); 
  
  if( !capture.isOpened() ) {
    printf("/t * Could not open the capture object \n");
    return -1;
  }
  printf("\t * Opened device \n");

  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on

  printf("\t * Common Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_PROP_FRAME_HEIGHT ) );
  
  // Set control panel
  int value;
  cv::namedWindow( mWindowName );
  // Set mouse callback 
  cv::setMouseCallback( mWindowName, onMouse, 0 );
  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Send point", sendPoint, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  // Loop
  for(;;) {
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }

    capture.retrieve( rgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    cv::imshow( mWindowName, rgbImg );

    capture.retrieve( pclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }

  } // end for
  
  return 0;
}


/**
 * @function startComm
 */
void startComm( int state, void* userdata ) {
  sns_init();
  sns_start();
  sns_chan_open( &mPoint_chan, gPoint_chan_name.c_str(), NULL );
  printf("\t [OK] Communication stablished and ready to go \n");
}

/**
 * @function send
 */
void sendPoint( int state, void* userdata ) {

  ach_status r;
  sns_msg_vector* msg = sns_msg_vector_heap_alloc(3);
  for( int i = 0; i < 3; ++i ) {
    msg->x[i] = mP[i];
  }
  r = ach_put( &mPoint_chan, msg, sns_msg_vector_size(msg) );
  if( r != ACH_OK ) {
    printf("\t [BAD] Error sending message \n");
  } else {
    printf("\t [GOOD] Message sent all right \n ");
  }
}



/**
 * @function onMouse
 * @brief Prints the depth of the clicked position
 */
static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  p = pclMap.at<cv::Point3f>(y,x);
  
  mP[0] = -p.x; mP[1] = p.y; mP[2] = p.z;
  printf( "\t * Depth clicked: %f %f %f \n", mP[0], mP[1], mP[2] );
 
}
