/**
 * @file pickObject
 * @brief Select an object with a click and send an ACH message with the said information
 * @author A. Huaman Q.
 */

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <ach.h>
#include <sns.h>

#include <Eigen/Core>

cv::Mat rgbImg;
cv::Mat pclMap;

ach_channel_t obj_pos_chan;
Eigen::Vector3d currentPoint;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void startComm( int state, void* userdata );
void sendMsg( int state, void* userdata );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI );
  //capture.open( cv::CAP_OPENNI );
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }

  // Set control panel
  cv::namedWindow( "BGR", cv::WINDOW_AUTOSIZE );
  //  cv::CreateTrackbar("track1", "BGR", &value, 255, NULL );
  
  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Send", sendMsg, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  

  // Set mouse callback 
  cv::setMouseCallback( "BGR", onMouse, 0 );

  // Loop
  for(;;) {

    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }

    capture.retrieve( rgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    cv::imshow( "BGR", rgbImg );
         
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
 * @function onMouse
 * @brief Stores the current position of clicked point and calculates the world position from the robot's kinematics
 */
static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  p = pclMap.at<cv::Point3f>(y,x);
  currentPoint << (double)p.x, (double)p.y, (double)p.z;

  std::cout << "\t * [INFO] Current point ready to send: "<< currentPoint.transpose() << std::endl;
  
}

/**
 * @function startComm
 */
void startComm( int state, void* userdata ) {

  printf("\t * Start communication \n");
  sns_init();
  sns_start();

  sns_chan_open( &obj_pos_chan, "obj-pos", NULL );  
  printf("\t * [OK] Communication stablished and ready to go \n");

}

/**
 * @function process
 */
void sendMsg( int state, void* userdata ) {

  // Send through channel
  double msg[3];
  for( int i =0; i < 3; ++i ) { msg[i] = currentPoint(i); }

  ach_status r;
  r = ach_put( &obj_pos_chan, msg, sizeof(msg) );
  if( r != ACH_OK ) {
    printf("\t * [BAD] Something bad happened while sending obj Pos \n");
  }

}

