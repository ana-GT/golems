/**
 * @file calibration_v0.cpp
 * @brief Grab RGB, depth and kinematics, spits out calibration
 * @author A. Huaman Q.
 */
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include "perception/marker_detection/crichton_markerDetector.h"
#include "global/crichton_global.h"
#include "perception/calibration/calibration_utils.h"
#include "perception/msgs/perception_msgs.h"

struct calib_pair{
  Eigen::Vector3d pk;
  Eigen::Vector3d pr;
  int id;
};

cv::Mat mRgbImg;
cv::Mat mDepthImg;
std::string mWindowName( "Calibration Crichton v1" );
ach_channel_t mMarker_robot_chan;

crichton_markerDetector mMd;
std::vector<calib_pair> mP[2];

std::vector<Eigen::Vector3d> mPk[2];
std::vector<Eigen::Vector3d> mPr[2];

std::vector<Eigen::Vector3d> mPam[2];
std::vector<int> mId[2];





int counter = 0;

/***********************/
/** FUNCTIONS          */
/***********************/
void grabSnapshot( int state, void* userdata );
void showSnapshot( int state, void* userdata );
void storeSnapshot( int state, void* userdata );
void discardSnapshot( int state, void* userdata );
void calibrate( int state, void* userdata );

static void onMouse( int event, int x, int y, int, void* );

bool openComm();
void update_robotInfo();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  // Open channels with marker-position information
  if( !openComm() ) { printf("Error opening comm \n"); return 1; }
  
  // Marker detector
  mMd.setMarkerSize(5);
  
  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 ); 
  
  if( !capture.isOpened() ) {
    std::cout << "\t * Could not open the capture object"<<std::endl;
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
  int value;
  cv::namedWindow( mWindowName );
  // Set mouse callback 
  cv::setMouseCallback( mWindowName, onMouse, 0 );

  // Set buttons
  cv::createTrackbar( "T1_tb", "T1", &value, 255, NULL, NULL );
  cv::createButton( "Grab snapshot", grabSnapshot, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Show snapshot", showSnapshot, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createTrackbar( "T2_tb", "T2", &value, 255, NULL, NULL );
  cv::createButton( "Store snapshot", storeSnapshot, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Discard snapshot", discardSnapshot, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createTrackbar( "T3_tb", "T3", &value, 255, NULL, NULL );
  cv::createButton( "Calibrate", calibrate, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  
  // Loop
  for(;;) {
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }

    capture.retrieve( mRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    capture.retrieve( mDepthImg, cv::CAP_OPENNI_POINT_CLOUD_MAP );
        
    update_robotInfo();
    mMd.detect( mRgbImg );

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
  printf("Depth: %f %f %f \n", -p.x, p.y, p.z); 
}

/**
 * @function openComm
 */
bool openComm() {

  // Set marker robot channel
  enum ach_status r;
  
  r = ach_open( &mMarker_robot_chan, gMarker_robot_chan_name.c_str(), NULL );
  
  return( r == ACH_OK );    
}



/**
 *
 */
void update_robotInfo() {

  struct sns_msg_marker_robot* msg = 0;
  size_t frame_size;
  ach_status r;
  
  r = sns_msg_local_get( &mMarker_robot_chan,
			 (void**)&msg,
			 &frame_size,
			 NULL, ACH_O_LAST );
  
  if( ACH_OK == r || ACH_MISSED_FRAME == r ) {

    for( int i = 0; i < 2; ++i ) {
      mPam[i].resize( msg->header.n );
      mId[i].resize( msg->header.n );
    }

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < mPam[i].size(); ++j ) {
	mPam[i][j] = Eigen::Vector3d( msg->u[j].x[i], msg->u[j].y[i], msg->u[j].z[i] );
	mId[i][j] = msg->u[j].id[i];	
      }
    }
    
    
  }
  msg = 0;

  
}

/**
 * @fuction grabSnapshot
 * @brief Store current information
 */
void grabSnapshot( int state, void* userdata ) {

  // Reset
  for( int i = 0; i < 2; ++i ) { mP[i].resize(0); }
  
  std::vector<calib_marker> markers;
  markers = mMd.getCalibMarkers();
  int ind;
  
  for( int i = 0; i < 2; ++i ) {
    for( int j = 0; j < mId[i].size(); ++j ) {

      
      // Check if id was found
      ind = -1;
      for( int k = 0; k < markers.size(); ++k ) {
	if( mId[i][j] == markers[k].id ) {
	  ind = k; break;
	}
      }

      if( ind != -1 ) {
	// Get position in Kinect coordinates
	cv::Point3f p; p = mDepthImg.at<cv::Point3f>( markers[ind].py, markers[ind].px );
	// Check if it is valid
	if( p.x == 0 && p.y == 0 && p.z == 0 ) { continue; }
	
	// If valid, get position in robot coordinates and store
	calib_pair cp;
	cp.pk = Eigen::Vector3d( -p.x, p.y, p.z );
	cp.pr = mPam[i][j];
	cp.id = mId[i][j];
	mP[i].push_back( cp );
      }
      
      
    }
  }
  
}

/**
 * @fuction showSnapshot
 */
void showSnapshot( int state, void* userdata ) {

  for( int i = 0; i < 2; ++i ) {
    printf("Arm [%d] validmarkers detected \n", i);
    for( int j = 0; j < mP[i].size(); ++j ) {
      printf("Id: %d - Pk: %f %f %f - Pr: %f %f %f \n", mP[i][j].id,
	     mP[i][j].pk(0), mP[i][j].pk(1), mP[i][j].pk(2),
	     mP[i][j].pr(0), mP[i][j].pr(1), mP[i][j].pr(2) );
    }
  }
}

/**
 * @function storeSnapshot
 */
void storeSnapshot( int state, void* userdata ) {
  for( int i = 0; i < 2; ++i ) {
    for( int j = 0; j < mP[i].size(); ++j ) {
      mPk[i].push_back( mP[i][j].pk );
      mPr[i].push_back( mP[i][j].pr );
    }
    printf("Points stored so far for arm %d : %d \n", i, mPk[i].size() );
  }
}

/**
 * @function discardSnapshot
 */
void discardSnapshot( int state, void* userdata ) {
  // Uh? Well, don't store, that is it
}

/**
 * @function calibrate
 */
void calibrate( int state, void* userdata ) {

  Eigen::Isometry3d Tf[2];
  
  for( int i = 0; i < 2; ++i ) {
    svd_method( mPk[i], mPr[i], mPk[i].size(), Tf[i] );

    std::cout << "Trk ["<<i<<"] from "<<mPk[i].size() <<" points: "<< std::endl;
    std::cout << Tf[i].matrix() << std::endl;   
  }
  
}

