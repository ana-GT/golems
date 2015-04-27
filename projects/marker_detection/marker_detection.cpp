/**
 * @file marker_detection.cpp
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sns.h>
#include <stdint.h>
#include <ach.h>

#include <msgs/perception_msgs.h>

#include "crichton_markerDetector.h"

ach_channel_t mImg_chan;
ach_channel_t mMarkers_chan;
cv::Mat mImg;
std::string mWindowName( "Marker Test" );

struct timespec t_now;
struct timespec t_timeout;

bool poll_image( cv::Mat &_img,
		 ach_channel_t* _chan );

crichton_markerDetector mMd;


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open channel
  sns_chan_open( &mImg_chan, "rgb-chan", NULL );
  sns_chan_open( &mMarkers_chan, "markers-chan", NULL );


  // Setup
  mMd.setMarkerSize(5);
  
  cv::namedWindow( mWindowName.c_str() );

  while( !sns_cx.shutdown ) {
    // Read message 
    if( poll_image( mImg, &mImg_chan ) == true ) {
    
      // Detect markers
      if( mMd.detect(mImg) ) {	
	// Send message
	
      }
      
      cv::imshow( mWindowName.c_str(), mImg );
                  
    } // polling

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * PRESSED 'q'. Finishing the program \n");
      return 1;	
    }
   
    // Release memory
    aa_mem_region_local_release();

    
  } // while

}

/**
 * @function poll_image
 */
bool poll_image( cv::Mat &_img,
		 ach_channel_t* _chan ) {
  
  ach_status_t r;
  size_t frame_size;
  void* buf = NULL;
  int counter;
  
  if( clock_gettime( ACH_DEFAULT_CLOCK, &t_now ) ) {
    SNS_LOG( LOG_ERR, "clock_gettime failed: '%s' \n", strerror(errno) );
    return 0;
  }
  
  t_timeout = sns_time_add_ns( t_now, 1000*1000*30 );
  
  r = sns_msg_local_get( _chan,
			 &buf,
			 &frame_size,
			 &t_timeout,
			 ACH_O_LAST | ACH_O_WAIT );
  
  switch(r) {
    
  case ACH_OK:
  case ACH_MISSED_FRAME: {

    struct sns_msg_rgb_img* msg = (struct sns_msg_rgb_img*) buf;
    if( frame_size == sns_msg_rgb_img_size(msg) ) {

      // Store 
      _img = cv::Mat( msg->height, msg->width, CV_8UC3 );
      counter = 0;
      for( size_t x = 0; x < msg->height; ++x ) {
	for( size_t  y = 0; y < msg->width; ++y ) {
	  _img.at<cv::Vec3b>(x,y)[0] = msg->u[counter].x;
	  _img.at<cv::Vec3b>(x,y)[1] = msg->u[counter].y;
	  _img.at<cv::Vec3b>(x,y)[2] = msg->u[counter].z;
	  counter++;	  
	} 
      } // end for
      return true;
    } // end if

  } break;
    
  } // end switch
    
  return false;
  
}
