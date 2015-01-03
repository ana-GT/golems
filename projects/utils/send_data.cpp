/**
 * @file send_data.cpp
 * @brief Send data from ain image using ACH (depth or rgb or both)
 * @author A. Huaman Q.
 * @date 2014/12/31
 */

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <sns.h>
#include <unistd.h>
#include <ach.h>

#include "msgs/perception_msgs.h"

#include <Eigen/Core>

cv::Mat mRgbImg;
cv::Mat mDepthImg;

char* mRgb_chan_name = "rgb-chan";
char* mDepth_chan_name = "depth-chan";

ach_channel_t mRgb_chan;
ach_channel_t mDepth_chan;

bool send_rgb = true;
bool send_depth = false;

int counter = 0;

/***********************/
/** FUNCTIONS          */
/***********************/
void sendData();
void setComm();
void printHelp( char* argv );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  //--------------------
  // 0. Logistics
  //--------------------
  int c;
  while((c = getopt(argc, argv, "r:d:h" )) != -1 ) {
    switch(c) {
    case 'h': { printHelp(argv[0]); return 1; } break;
    case 'r': { mRgb_chan_name = optarg; send_rgb = true; } break;
    case 'd': { mDepth_chan_name = optarg; send_depth = true; } break; 
    } // end switch
  } // end while

  // Set communication
  setComm();

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 ); 
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }
  printf("\t * Opened device \n");

  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 1.0 );
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on

  printf("\t * Only Image dimensions: (%f,%f) Depth dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ),
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ) );

  // Set comm
  setComm();
  
  // Loop
  for(;;) {
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }
    
    capture.retrieve( mRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    capture.retrieve( mDepthImg, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    
    // Send message
    sendData();

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }

  } // end for
  
  return 0;
}


/**
 * @function setComm
 */
void setComm() {
  if( send_rgb ) {
    sns_chan_open( &mRgb_chan, mRgb_chan_name, NULL );
  }
  if( send_depth ) {
    sns_chan_open( &mDepth_chan, mDepth_chan_name, NULL );
  }
}

/**
 * @function sendData
 */
void sendData( ) {

  // Create a RGB image
  if( send_rgb ) {
    sns_msg_rgb_img* msg = sns_msg_rgb_img_alloc( mRgbImg.cols, mRgbImg.rows );
    msg->width = mRgbImg.cols;
    msg->height = mRgbImg.rows;
    
    int i = 0;
    for(int y = 0; y < mRgbImg.rows; y++) {
      for(int x = 0; x < mRgbImg.cols; x++) {
	
	msg->u[i].x = (uint8_t) mRgbImg.at<cv::Vec3b>(y,x)[0];
	msg->u[i].y = (uint8_t) mRgbImg.at<cv::Vec3b>(y,x)[1];
	msg->u[i].z = (uint8_t) mRgbImg.at<cv::Vec3b>(y,x)[2];
	i++;
      }
    }
    
    ach_status_t r = ach_put( &mRgb_chan, msg, sns_msg_rgb_img_size(msg) );
    SNS_REQUIRE( ACH_OK == r, "Could not put frame: %s\n", ach_result_to_string(r) );
    
  }

  if( send_depth ) {
    sns_msg_depth_img* msg = sns_msg_depth_img_alloc( mDepthImg.cols, mDepthImg.rows );
    msg->width = mDepthImg.cols;
    msg->height = mDepthImg.rows;
    
    int i = 0;
    for(int y = 0; y < mDepthImg.rows; y++) {
      for(int x = 0; x < mDepthImg.cols; x++) {	
	msg->u[i].x = mDepthImg.at<cv::Vec3f>(y,x)[0];
	msg->u[i].y = mDepthImg.at<cv::Vec3f>(y,x)[1];
	msg->u[i].z = mDepthImg.at<cv::Vec3f>(y,x)[2];
	i++;
      }
    }

    ach_status_t r = ach_put( &mDepth_chan, msg, sns_msg_depth_img_size(msg) );
    SNS_REQUIRE( ACH_OK == r, "Could not put frame: %s\n", ach_result_to_string(r) );    
  }
  
    
  counter++;
}

/**
 * @function printHelp
 */
void printHelp( char* argv ) {
}
