/**
 * @file see_module_v2
 * @brief Captures a snapshot, segments and select the type of primitive you want to 
use to represent the objects
 * @brief It can be controlled by msgs from server
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
#include "tabletop_segmentation/tabletop_segmentation.h"

#include "msgs/perception_msgs.h"
#include <communication/msgs/server_msgs.h>

#include <global/crichton_global.h>
#include <global/fsa_data.h>

#include <stdlib.h>
#include <time.h>

#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>


// SQ, BB
#include "refresher_utils/Refresher_utils.h"
#include "sq_fitting/SQ_utils.h"
#include "sq_fitting/SQ_fitter.h"
#include "tabletop_symmetry/mindGapper.h"


/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string windowName = std::string("See Module v2");
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
bool mChanReady = false;
bool mSendState;

// CHANNELS
ach_channel_t mObj_param_chan; // Channel to send object param to planner
ach_channel_t mServer2See_chan; // Channel to receive commands from server
ach_channel_t mSee2Server_chan; // Channel to send responses to server


/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void process( int state, void* userdata );

void startComm( int state, void* userdata );
void setSQ( int state, void* userData );
void send( int state, void* userData );
void fit_SQ( pcl::PointCloud<pcl::PointXYZRGBA> _cluster,
	     double dim[3], double trans[3], double rot[3], double e[2] );

void drawSegmented();
void getPixelClusters();

void pollChan(); // Check communication with server

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
  
  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on
  
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

    if( mChanReady ){ pollChan(); }
    

    char k = cv::waitKey(30);
    if( k == 'q' ) { break; }

  } // end for
  
  return 0;
}

/**
 * @function pollChan
 * @brief Poll server channel for commands
 */
void pollChan() {

  // Poll server channel
  ach_status_t r;
  size_t frame_size;
  sns_msg_server_1 server_msg;
  sns_msg_process_1 see_msg;
  bool result;
  
  r = ach_get( &mServer2See_chan, &server_msg,
	       sizeof(server_msg), &frame_size, NULL, ACH_O_LAST );

  if( r == ACH_OK && !sns_msg_is_expired( &server_msg.header, NULL ) ) {

    if( server_msg.msg_type == START_SEEING_MSG ) {

      process(0, NULL );
      if( isSegmentedFlag ) {
	drawSegmented();
      }
      
      // Send first object
      selectedSegmentedCloud = 0;

      // Send it to planning
      send( 0, NULL );
      usleep(0.5*1e6);

      // Send message replying
      see_msg.msg_type = FINISHED_SEEING_MSG;
      see_msg.flag = mSendState;
      sns_msg_set_time( &see_msg.header, NULL, 0.2*1e9 );
      r = ach_put( &mSee2Server_chan, &see_msg, sizeof(see_msg) );         
    }

    
  }
}

/**
 * @function startComm
 * @brief Open/Flush channels for communication with planner and server
 */
void startComm( int state, void* userdata ) {

  ach_status_t r;

  // Open channels
  r = ach_open( &mObj_param_chan, gObj_param_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { printf("[ERROR] COULD NOT OPEN OBJ PARAM CHAN"); }
  r = ach_open( &mServer2See_chan, gServer2See_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { printf("[ERROR] COULD NOT OPEN SERVER-2-SEE CHAN"); }
  r = ach_open( &mSee2Server_chan, gSee2Server_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { printf("[ERROR] COULD NOT SEE-2-SERVER CHAN"); }

  // Flush channels
  ach_flush( &mObj_param_chan );
  ach_flush( &mServer2See_chan );
  ach_flush( &mSee2Server_chan );
  
  mChanReady = true;
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
 * @function send
 */
void send( int state, void* userData ) {

  if( selectedSegmentedCloud < 0 || selectedSegmentedCloud >= clusters.size() ) {
    printf("--> ERROR: Did not select a cloud? \n");
    mSendState = false; return;
  }
  
  double dim[3]; double trans[3]; double rot[3]; double e[2];  
  SQ_parameters_msg msg;
  
  fit_SQ( clusters[selectedSegmentedCloud], dim, trans, rot, e );

  // Print parameters
  std::cout << "Parameters: " << e[0] << " and " << e[1] << std::endl;
  
  
  for(int i = 0; i < 3; ++i ) { msg.param.dim[i] = dim[i]; }
  for( int i = 0; i < 3; ++i ) { msg.param.trans[i] = trans[i]; }
  for( int i = 0; i < 3; ++i ) { msg.param.rot[i] = rot[i]; }
  for( int i =0; i < 2; ++i ) { msg.param.e[i] = e[i]; }
  
  // Generate mesh
  pcl::PointCloud<pcl::PointNormal>::Ptr pn( new pcl::PointCloud<pcl::PointNormal>() );
  sampleSQ_uniform_pn( dim[0], dim[1], dim[2], e[0], e[1], 50, pn );
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(5);
  poisson.setInputCloud(pn);
  pcl::PolygonMesh mesh;
  poisson.reconstruct( mesh );

  int number = rand() % 1000;
  char outputName[255];
  sprintf( outputName, "%s/mesh_%d.ply", gPicturesPath.c_str(),  number);
  std::cout << "Mesh: "<< outputName << std::endl;
  pcl::io::savePLYFile(outputName, mesh);
  
  msg.mesh_generated = true;
  strcpy( msg.mesh_filename, outputName );
  
  ach_status_t r = ach_put( &mObj_param_chan, &msg, sizeof(msg) );
  
  mSendState = (r  == ACH_OK );
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
    //mg.complete( completed );

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
