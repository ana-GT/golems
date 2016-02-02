/**
 * @file eye_v1
 * @brief Captures a snapshot, segments and select the type of primitive you want to use to represent the objects
 * @author A. Huaman Q.
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
#include "perception/tabletop_segmentation/tabletop_segmentation.h"
#include "perception/msgs/perception_msgs.h"

#include <global/crichton_global.h>

// SQ, BB
#include "perception/pointcloud_tools/refresher_utils/Refresher_utils.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"
#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper.h"


/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string windowName = std::string("Crichton Eye v1");
cv::Mat rgbImg;
cv::Mat pclMap;

ach_channel_t segmented_cloud_chan;
Eigen::Vector3d currentPoint;
bool isSegmentedFlag = false;
double f;

int selectedSegmentedCloud = -1;
std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clusters;
std::vector<cv::Vec3b> colors;
std::vector< std::vector<Eigen::Vector2d> > pixelClusters;
std::vector<Eigen::Vector2d> clusterCentroids;
std::vector<double> mTableCoeffs;

ach_channel_t mObj_param_chan;

enum {
  BB_FLAG = 86,
  SQ_FLAG = 87,
};

int mPrimFlag;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void process( int state, void* userdata );

void startComm( int state, void* userdata );
void setBB( int state, void* userData );
void setSQ( int state, void* userData );
void send( int state, void* userData );
void fit_BB( pcl::PointCloud<pcl::PointXYZRGBA> _cluster,
	     double dim[3], double trans[3], double rot[3] );
void fit_SQ( pcl::PointCloud<pcl::PointXYZRGBA> _cluster,
	     double dim[3], double trans[3], double rot[3], double e[2] );

void drawSegmented();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Initialization
  srand( time(NULL) );
  
  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 );
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }

  printf("\t * Opened device \n");
  
  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on
  
  printf("\t * Common Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_PROP_FRAME_HEIGHT ) );
  

  // Set control panel
  cv::namedWindow( windowName, cv::WINDOW_AUTOSIZE );
  
  int value;
  cv::createTrackbar("track1", windowName.c_str(), &value, 255, NULL, NULL );

  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  cv::createButton( "Bounding Box", setBB,
		    NULL, cv::QT_RADIOBOX,
		    false );
  cv::createButton( "Superquadric", setSQ,
		    NULL, cv::QT_RADIOBOX,
		    false );

  cv::createButton( "Send", send, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  
  // Set mouse callback 
  cv::setMouseCallback( windowName, onMouse, 0 );

  // Loop
  f = (float)capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );

  for(;;) {
    
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }
    
    capture.retrieve( rgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    if( isSegmentedFlag ) {
      drawSegmented();
    }
    cv::imshow( windowName, rgbImg );
         
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
 * @function startComm
 */
void startComm( int state, void* userdata ) {
  sns_init();
  sns_start();
  sns_chan_open( &mObj_param_chan, gObj_param_chan_name.c_str(), NULL );
  printf("\t [OK] Communication stablished and ready to go \n");
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
  p = pclMap.at<cv::Point3f>(y,x);
  currentPoint << (double)p.x*-1, (double)p.y, (double)p.z;

  std::cout << "\t * [INFO] Current point: "<< currentPoint.transpose() << std::endl;

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
}


/**
 * @function setXX
 * @brief Set flag representation of one-view pointcloud
 */
void setBB( int state, void* userData ) { mPrimFlag = BB_FLAG; }
void setSQ( int state, void* userData ) { mPrimFlag = SQ_FLAG; }

/**
 * @function send
 */
void send( int state, void* userData ) {

  if( selectedSegmentedCloud < 0 || selectedSegmentedCloud >= clusters.size() ) {
    printf("--> ERROR: Did not select a cloud? \n");
    return;
  }
  
  double dim[3]; double trans[3]; double rot[3]; double e[2];  
  sns_msg_vector* msg;
  
  switch( mPrimFlag ) {
  case BB_FLAG: {
    fit_BB( clusters[selectedSegmentedCloud], dim, trans, rot );
    msg = sns_msg_vector_heap_alloc(9);
    msg->header.n = 9;
    for(int i = 0; i < 3; ++i ) { msg->x[i] = dim[i]; }
    for( int i = 0; i < 3; ++i ) { msg->x[i+3] = trans[i]; }
    for( int i = 0; i < 3; ++i ) { msg->x[i+6] = rot[i]; }
  } break;
  case SQ_FLAG: {
    fit_SQ( clusters[selectedSegmentedCloud], dim, trans, rot, e );
    // Store segmented cloud
    pcl::io::savePCDFileASCII( "segmented_sq.pcd", clusters[selectedSegmentedCloud] );
    std::cout << "Stored pointcloud "<< std::endl;
    // Print parameters
    std::cout << "Parameters: " << e[0] << " and " << e[1] << std::endl;

    msg = sns_msg_vector_heap_alloc(11);
    msg->header.n = 11;
    for(int i = 0; i < 3; ++i ) { msg->x[i] = dim[i]; }
    for( int i = 0; i < 3; ++i ) { msg->x[i+3] = trans[i]; }
    for( int i = 0; i < 3; ++i ) { msg->x[i+6] = rot[i]; }
    for( int i =0; i < 2; ++i ) { msg->x[i+9] = e[i]; }
  } break;

  }

  ach_status_t r = ach_put( &mObj_param_chan, msg, sns_msg_vector_size(msg) );
  if( r != ACH_OK ) {
    printf("\t [BAD] Error sending message \n");
  } else {
    printf("\t [GOOD] Message sent all right \n ");
  }
}

/**
 * @function fit_BB
 * @brief 
 */
void fit_BB( pcl::PointCloud<pcl::PointXYZRGBA> _cluster,
	     double _dim[3], double _trans[3], double _rot[3] ) {

  printf("Fit bounding boxes \n");

  Eigen::Isometry3d Tf;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
  p->points.resize( _cluster.points.size() );
  for(int j = 0; j < _cluster.points.size(); ++j ) {
    p->points[j].x = _cluster.points[j].x;
    p->points[j].y = _cluster.points[j].y;
    p->points[j].z = _cluster.points[j].z;
  }
  p->width = 1; p->height = p->points.size();
  
  // Get Bounding Box
  pointcloudToBB( p, _dim, Tf );

  _trans[0] = Tf.translation()(0);
  _trans[1] = Tf.translation()(1);
  _trans[2] = Tf.translation()(2);
  Eigen::Vector3d rpy; rpy = (Tf.linear()).eulerAngles(2,1,0);
  _rot[0] = rpy(2); _rot[1] = rpy(1); _rot[2] = rpy(0);
  
  printf("--> Bounding box with dimensions: %f %f %f \n", _dim[0], _dim[1], _dim[2] );
  
}

/**
 * @function fit_SQ
 */
void fit_SQ( pcl::PointCloud<pcl::PointXYZRGBA> _cluster,
	     double _dim[3], double _trans[3],
	     double _rot[3], double _e[2] ) {

  // Generate a mirror version of the pointcloud
  mindGapper<pcl::PointXYZRGBA> mg;
  mg.setTablePlane( mTableCoeffs );
  mg.setFittingParams();
  mg.setDeviceParams();
  
  // Fit pointcloud to superquadric
  SQ_fitter< pcl::PointXYZ > fitter;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr completed( new pcl::PointCloud<pcl::PointXYZRGBA>() );
    *completed = _cluster;
    mg.reset();
    mg.complete( completed );

    pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
    p->points.resize( completed->points.size() );
    for(int j = 0; j < completed->points.size(); ++j ) {
      p->points[j].x = completed->points[j].x;
      p->points[j].y = completed->points[j].y;
      p->points[j].z = completed->points[j].z;
    }
    p->width = 1; p->height = p->points.size();

    fitter.setInputCloud( p );
    if( fitter.fit( 0.03, 0.005, 5, 0.1 ) ) {

      SQ_parameters p;
      fitter.getFinalParams( p );
      for( int i = 0; i < 3; ++i ) { _dim[i] = p.dim[i]; }
      for( int i = 0; i < 3; ++i ) { _trans[i] = p.trans[i]; }
      for( int i = 0; i < 3; ++i ) { _rot[i] = p.rot[i]; }
      for( int i = 0; i < 2; ++i ) { _e[i] = p.e[i]; }
      printf("--> SQ dimensions: %f %f %f \n", _dim[0], _dim[1], _dim[2] );
      printf("--> SQ position: %f %f %f \n", _trans[0], _trans[1], _trans[2] );
    }
  
}


/**
 * @function process
 */
void process( int state, 
	      void* userdata ) {


  // Get organized pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );
  cv::Point3f p;
  cv::Vec3i col;
  pcl::PointXYZRGBA P;

  int width = pclMap.cols;
  int height = pclMap.rows;

  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false; // some NaN can be found
  cloud->points.resize( width * height );

  for( size_t j = 0; j < height; ++j ) {
    for( size_t i = 0; i < width; ++i ) {

      p = pclMap.at<cv::Point3f>(j,i);
      P.x = p.x*-1; P.y = p.y; P.z = p.z;
      P.r = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[2];
      P.g = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[1]; 
      P.b = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[0];
      P.a = 255;
      cloud->points[width*j + i] = P;
      
    }
  }

  // Segment
  TabletopSegmentor<pcl::PointXYZRGBA> tts;
  
  tts.processCloud( cloud );
  mTableCoeffs = tts.getTableCoeffs();
  int n = tts.getNumClusters();

  // Set segmented variables
  isSegmentedFlag = true;
  clusters.resize(n);
  colors.resize(n);
  
  for( int i = 0; i < n; ++i ) {
    clusters[i] = tts.getCluster(i);

    cv::Vec3b def;
    def(0) = rand() % 255; def(1) = rand() % 255; def(2) = rand() % 255;  
    colors[i] = def;
  }
  getPixelClusters();
  
}


/**
 * @function drawSegmented
 */
void drawSegmented() {
  for( int i = 0; i < pixelClusters.size(); ++i ) {
    for( int j = 0; j < pixelClusters[i].size(); ++j ) {
      rgbImg.at<cv::Vec3b>( pixelClusters[i][j](1), pixelClusters[i][j](0) ) = colors[i];
    }
  }
  
}

/**
 * @function getPixelClusters
 */
void getPixelClusters() {

  pixelClusters.resize( clusters.size() );
  clusterCentroids.resize( clusters.size() );
  
  for( int i = 0; i < pixelClusters.size(); ++i ) {
    pixelClusters[i].resize(0);
  }
  
  int u, v;
  int width, height;
  double X, Y, Z; 
  int sum_u; int sum_v;

  // Get (u,v) pixel of clusters  
  width = rgbImg.cols;
  height = rgbImg.rows;

  for( int i = 0; i < clusters.size(); ++i ) {
   
    sum_u = 0;
    sum_v = 0;

    for( int j = 0; j < clusters[i].points.size(); ++j ) {

      X = clusters[i].points[j].x;
      Y = clusters[i].points[j].y;
      Z = clusters[i].points[j].z;
      
      u = width/2 - (int)(X*f/Z);
      v = height/2 -(int)(Y*f/Z);
      
      pixelClusters[i].push_back( Eigen::Vector2d(u,v) );


      sum_u += u;
      sum_v += v;
    }
    
    Eigen::Vector2d ct;
    ct << (double)(sum_u)/clusters[i].points.size(), (double)(sum_v)/clusters[i].points.size();
    clusterCentroids[i] = ct;

  }

  
}
