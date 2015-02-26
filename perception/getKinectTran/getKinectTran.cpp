/**
 * @file getKinectTran
 * @brief Get transformation from Kinect w.r.t. the robot world origin
 * @brief using a bunch of correspondences and SVD for minimization
 */

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <fstream>

#include <ach.h>
#include <sns.h>

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pk;
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Pw;

cv::Mat rgbImg;
cv::Mat pclMap;

ach_channel_t ee_pos_chan;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void startComm( int state, void* userdata );
void process( int state, void* userdata );
void icpApproach();
void svdApproach();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI );
//  capture.set( cv::CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv::CAP_OPENNI_SXGA_15HZ );
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
  cv::createButton( "Process", process, 
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
  Eigen::Vector3d pk;

  p = pclMap.at<cv::Point3f>(y,x);
  pk(0) = (double)p.x; pk(1) = (double)p.y; pk(2) = (double)p.z;

  // Get (X,Y,Z) from robot kinematics
  Eigen::Vector3d pw;
  double msg[3];
  size_t frame_size;
  ach_status r;
  
  r = ach_get( &ee_pos_chan,
	       msg,
	       sizeof(msg),
	       &frame_size, NULL, ACH_O_LAST );

  if( r != ACH_MISSED_FRAME && r != ACH_OK ) {
    printf("\t * [BAD] Did not receive updated EE pos - NO STORING POINT \n");
    return;
  }
  
  for( int i = 0; i < 3; ++i ) { pw(i) = msg[i]; }
  printf( "Pixel: %d, %d; \n", x, y );
  printf( "Pk[%d] << %f, %f, %f; Pr[%d] << %f, %f, %f;\n", Pk.size(), 
	  pk(0), pk(1), pk(2), Pw.size(), pw(0), pw(1), pw(2) );

  // Store it
  Pk.push_back(pk);
  Pw.push_back(pw);
  printf("*\t * Stored %d points so far \n", Pk.size() );
  
}

/**
 * @function startComm
 */
void startComm( int state, void* userdata ) {

  printf("\t * Start communication \n");
  sns_init();
  sns_start();

  sns_chan_open( &ee_pos_chan, "ee-pos", NULL );  
  printf("\t * [OK] Communication stablished and ready to go \n");

}

/**
 * @function process
 * @brief Store data
 */
void process( int state, void* userdata ) {

  std::ofstream file("data.txt");

  if(file.is_open())
  {
    for(int i =0; i < Pk.size(); i++)
    {
	file << Pk[i](0) << " " << Pk[i](1) << " " << Pk[i](2) << " ";
	file << Pw[i](0) << " " << Pw[i](1) << " " << Pw[i](2) << " " << std::endl;
    }
	file.close();
  }



}


