/**
 * @file send_data.cpp
 * @brief Send image data using ACH
 * @author A. Huaman Q.
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
char mRgb_chan_name[50] = "image-chan";
ach_channel_t mRgb_chan;

int counter = 0;

/***********************/
/** FUNCTIONS          */
/***********************/
void sendData();
void setComm();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

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
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, 1.0 ); // on

  printf("\t * Common Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_PROP_FRAME_HEIGHT ) );

  printf("\t * Only Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ) );

  printf("\t * Only Depth dimensions: (%f,%f) \n",
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
  sns_chan_open( &mRgb_chan, mRgb_chan_name, NULL );
}

/**
 * @function sendData
 */
void sendData( ) {

  // Create a message
  sns_msg_rgb_img* msg = sns_msg_rgb_img_alloc( mRgbImg.cols, mRgbImg.rows );
  msg->width = mRgbImg.cols;
  msg->height = mRgbImg.rows;

  int i = 0;
  for(int x = 0; x < mRgbImg.rows; x++) {
    for(int y = 0; y < mRgbImg.cols; y++) {
      
      msg->u[i].x = (uint8_t) mRgbImg.at<cv::Vec3b>(x,y)[0];
      msg->u[i].y = (char) mRgbImg.at<cv::Vec3b>(x,y)[1];
      msg->u[i].z = (char) mRgbImg.at<cv::Vec3b>(x,y)[2];
      i++;
    }
  }

  ach_status_t r = ach_put( &mRgb_chan, msg, sns_msg_rgb_img_size(msg) );
  SNS_REQUIRE( ACH_OK == r, "Could not put frame: %s\n", ach_result_to_string(r) );
    
  counter++;
}

