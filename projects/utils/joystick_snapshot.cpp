#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <unistd.h>

#include <sns.h>  
#include <ach.h>

std::string gWindowName = std::string("Joystick snapshot");
cv::Mat gRgbImg;
ach_channel_t gJoystick_chan;


/**
 * @function checkChan
 */
bool checkChan() {

  sns_init();
  sns_start();

  ach_status_t r;

  r = ach_open(&gJoystick_chan, "joy_state", NULL );
  return ( r == ACH_OK );
}

/**
 * @function pollChan
 */
bool pollChan() {

  ach_status_t r;
  size_t frame_size;
  sns_msg_joystick* msg = 0;
  
  r = sns_msg_local_get( &gJoystick_chan, (void**)&msg,
			 &frame_size, NULL, ACH_O_LAST );
  
  double a[2];
  if( r == ACH_OK && !sns_msg_is_expired( &msg->header, NULL ) ) {
    
    a[0] = msg->axis[0];
    a[1] = msg->axis[1];
    if( a[0] != 0 || a[1] != 0 ) {
      return true;
    } else {
      return false;
    }
  } // end if

  return false;

}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Check if joystick is publishing
  if( !checkChan() ) {
    printf("Channel not started. Start joystick channel \n");
    return -1;
  }

  // Start camera
  cv::VideoCapture capture( -1 );
  if( !capture.isOpened() ) {
    printf("\t [ERROR] Capture object could not be opened \n");
    return -1;
  }


  // Constantly read the states of joystick
  for(;;) {
    
    if(!capture.grab() ) {
      printf("\t [ERROR] Could not grab a frame \n");
      return 1;
      continue;
    }

    capture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    cv::imshow( gWindowName, gRgbImg );
   
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t [PRESSED ESC] Finished the program \n");
      break;
    }

    // If axis 0/1 are moved, take a picture
    if( pollChan() ) {
      printf("Waiting 3 seconds to take picture \n");
      usleep(3*1e6);
      printf("Saving image \n");
      cv::imwrite( "takePicture.png", gRgbImg );
    }

  } // end for





}
