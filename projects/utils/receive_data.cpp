 /**
 * @file receive_data.cpp
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
std::string mWindowName( "Receive data" );

char* mRgb_chan_name = "rgb-chan";

ach_channel_t mRgb_chan;

int counter = 0;

/***********************/
/** FUNCTIONS          */
/***********************/
bool receiveData();
void setComm();
void printHelp( char* argv );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  //--------------------
  // 0. Logistics
  //--------------------

  // Set communication
  setComm();
  // Set window
  cv::namedWindow( mWindowName );
  
  // Loop
  for(;;) {
    
    // Receive message
    if( receiveData() ) {
      cv::imshow( mWindowName, mRgbImg );
    }

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
 * @function receiveData
 */
bool receiveData( ) {

  struct sns_msg_rgb_img* msg;
  size_t frame_size;
  ach_status r;
  
  r = sns_msg_local_get( &mRgb_chan,
			 (void**) &msg,
			 &frame_size,
			 NULL, ACH_O_LAST );

  if( ACH_OK == r || ACH_MISSED_FRAME == r ) {


    int width = msg->width;
    int height = msg->height;

    struct sns_msg_rgb_img* img = sns_msg_rgb_img_alloc( width, height );
    img = (sns_msg_rgb_img*) msg;
    
    mRgbImg = cv::Mat( height, width, CV_8UC3 );
    
    int i = 0;
    for(int y = 0; y < height; y++) {
      for(int x = 0; x < width; x++) {
	
	mRgbImg.at<cv::Vec3b>(y,x)[0] = (uint8_t) img->u[i].x;
	mRgbImg.at<cv::Vec3b>(y,x)[1] = (uint8_t) img->u[i].y;
	mRgbImg.at<cv::Vec3b>(y,x)[2] = (uint8_t) img->u[i].z;
	i++;
      }
    }
    
    
    return true;
  } else {
    return false;
  }


}

/**
 * @function printHelp
 */
void printHelp( char* argv ) {
}
