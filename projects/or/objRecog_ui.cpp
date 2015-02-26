/**
 * @file objRecog_ui
 * @brief Object recognition user interface
 * @author A. Huaman Quispe
 */
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ach.h>
#include <sns.h>

#include <Eigen/Core>
#include <stdint.h>

// Object recognition
#include <obj_recog/object_recognition.h>


/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string mWindowName = std::string("Crichton Recognizer");
cv::Mat mRgbImg;
cv::Mat mPclMap;

ObjectRecognition* mOr;


/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void loadModels( int state, void* userdata );
void recognize( int state, void* userdata );
void printHelp( char* _argv0 );


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Initialization
  srand( time(NULL) );
  
  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 );
  
  if( !capture.isOpened() ) {
    std::cout << "\t * Could not open the capture object"<<std::endl;
    return -1;
  }

  printf("\t * Opened device \n");
  
  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on
  
  printf("\t * Common Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_PROP_FRAME_HEIGHT ) );
  

  // Set control panel
  cv::namedWindow( mWindowName,
		   cv::WINDOW_AUTOSIZE );
  
  int value;
  cv::createTrackbar( "track1",
		      mWindowName.c_str(),
		      &value, 255,
		      NULL, NULL );

  cv::createButton( "Load Models", loadModels, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Recognize", recognize, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
    
  
  // Set mouse callback 
  cv::setMouseCallback( mWindowName, onMouse, 0 );

  // Loop
  for(;;) {
    
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }
    
    capture.retrieve( mRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    cv::imshow( mWindowName, mRgbImg );
         
    capture.retrieve( mPclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }
    
  } // end for
  
  return 0;
  
}

/**
 * @function loadModels
 */
void loadModels( int state, void* userdata ) {

  // Load parameters
  printf("DEBUG: Loading models \n");
  ObjectRecognitionParameters params;
  params.loadParams("/home/ana/Research/golems/projects/or/recog_params.json");
  mOr = new ObjectRecognition( params );

  std::cout << "DEBUG: \n"<< params;
  
  // Load models (so-called exemplars)
  mOr->populateDatabase( "/home/ana/Research/golems/projects/or/database_milk.json" );
  
  
  printf("DEBUG: Loaded parameters all right \n");

  //  mOr->populateDatabase( examplefilenames);
  
  
}

/**
 * @function recognize
 */
void recognize( int state, void* userdata ) {
  printf("Recognize \n");

  // FInd the object exemplar that best matches the query
  //mOr.recognizeAndAlighPoints( query );
}


/**
 * @function printHelp
 */
void printHelp( char* _argv0 ) {
    printf("\t Usage: %s \n", _argv0 );
    printf("\t -h : This help \n");
}

/**
 * @function onMouse
 * @brief Stores the current position of clicked point 
 */
static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  p = mPclMap.at<cv::Point3f>(y,x);
  Eigen::Vector3d currentPoint;
  currentPoint << (double)p.x*-1, (double)p.y, (double)p.z;

  std::cout << "\t * [INFO] Current point: "<< currentPoint.transpose() << std::endl;
  /*
  // Check what segmented object is selected
  if( clusterCentroids.size() > 0 ) {
    Eigen::Vector2d p; p << (double)x, (double) y;
    double dist;
    double minDist = 1000; int minInd = -1;
    for( int i = 0; i < clusterCentroids.size(); ++i ) {
      dist = ( p - clusterCentroids[i] ).norm();
      if( dist < minDist ) {
	minDist = dist; minInd = i; 
      }
    }
    
    selectedSegmentedCloud = minInd;

    std::cout << "\t [INFO] Segmented cloud to send: "<< selectedSegmentedCloud << std::endl;
  } // end if

  */
}
