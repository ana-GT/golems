/**
 * @file test_alvar3Daccuracy.cpp
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sns.h>
#include <stdint.h>
#include <ach.h>

#include <perception/msgs/perception_msgs.h>

#include <perception/marker_detection/crichton_markerDetector.h>

ach_channel_t mImg_chan;
ach_channel_t mDepth_chan;
cv::Mat mImg, mDepth;

std::string mWindowName( "test marker 3D accuracy" );

struct timespec t_now;
struct timespec t_timeout;

bool poll_image( cv::Mat &_img,
		 ach_channel_t* _chan );
bool poll_depth( cv::Mat &_depth,
		 ach_channel_t* _chan );

crichton_markerDetector mMd;


static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  p = mDepth.at<cv::Point3f>(y,x);
  printf("Depth: %f %f %f \n", p.x, p.y, p.z);
 
}


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open channel
  sns_chan_open( &mImg_chan, "rgb-chan", NULL );
  sns_chan_open( &mDepth_chan, "depth-chan", NULL );


  // Setup
  mMd.setMarkerSize(5);
  
  cv::namedWindow( mWindowName.c_str() );

  // Set mouse callback 
  cv::setMouseCallback( mWindowName, onMouse, 0 );
  
  while( !sns_cx.shutdown ) {
    // Read message 
    if( poll_image( mImg, &mImg_chan ) == true &&
	poll_depth( mDepth, &mDepth_chan ) == true ) {
      
      // Detect markers
      if( mMd.detect(mImg) ) {
	
	// Compare with depth from depth channel
	std::vector<calib_marker> markers;
	markers = mMd.getCalibMarkers();

	for( int i = 0; i < markers.size(); ++i ) {
	  int px, py;
	  px = (int)markers[i].px; py = (int)markers[i].py;
	  printf("[%d] Px: %d Py: %d \n", i, px, py);
	  printf("[%d] Marker 3D location detected: %f %f %f \n", i, markers[i].xc, markers[i].yc, markers[i].zc );
	  printf("[%d]Location from depth info: %f %f %f \n", i, mDepth.at<cv::Vec3f>(py,px)[0],
		 mDepth.at<cv::Vec3f>(py,px)[1], mDepth.at<cv::Vec3f>(py,px)[2]);
	}
	
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
      for( size_t y = 0; y < msg->height; ++y ) {
	for( size_t  x = 0; x < msg->width; ++x ) {
	  _img.at<cv::Vec3b>(y,x)[0] = msg->u[counter].x;
	  _img.at<cv::Vec3b>(y,x)[1] = msg->u[counter].y;
	  _img.at<cv::Vec3b>(y,x)[2] = msg->u[counter].z;
	  counter++;	  
	} 
      } // end for
      return true;
    } // end if

  } break;
    
  } // end switch
    
  return false;
  
}


/**
 * @function poll_depth
 */
bool poll_depth( cv::Mat &_depth,
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

    struct sns_msg_depth_img* msg = (struct sns_msg_depth_img*) buf;
    if( frame_size == sns_msg_depth_img_size(msg) ) {
      
      // Store 
      _depth = cv::Mat( msg->height, msg->width, CV_32FC3 );
      counter = 0;
      for( size_t y = 0; y < msg->height; ++y ) {
	for( size_t  x = 0; x < msg->width; ++x ) {
	  _depth.at<cv::Vec3f>(y,x)[0] = msg->u[counter].x;
	  _depth.at<cv::Vec3f>(y,x)[1] = msg->u[counter].y;
	  _depth.at<cv::Vec3f>(y,x)[2] = msg->u[counter].z;
	  counter++;	  
	} 
      } // end for
      return true;
    } // end if

  } break;
    
  } // end switch
    
  return false;
  
}
