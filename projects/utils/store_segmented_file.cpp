/**
 * @file store_segmented_file.cpp
 * @brief Store a segmented pointcloud + table normal in a file, till the program is killed (multiple entries)
 * @date July 4th, 2015
 * @author A. Huaman Q.
 */
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ach.h>
#include <sns.h>

#include <Eigen/Core>
#include <stdint.h>
#include "perception/tabletop_segmentation/tabletop_segmentation.h"


#include <stdlib.h>
#include <time.h>
#include <fstream>

#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>

// SQ, BB
#include "perception/pointcloud_tools/refresher_utils/Refresher_utils.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"
#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper.h"


typedef pcl::PointXYZRGBA PointTa;

/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string gWindowName = std::string("Store segmented cloud in file");
cv::Mat gRgbImg;
cv::Mat gPclMap;

Eigen::Vector3d gCurrentPoint;
bool gIsSegmentedFlag = false;
double gF;

int gCounter;
//std::ofstream gOutput;
std::ofstream gOutput( "file.txt", std::ofstream::out );

int gSelectedSegmentedCloud = -1;
std::vector<pcl::PointCloud<PointTa> > gClusters;
std::vector<cv::Vec3b> gColors;
std::vector< std::vector<Eigen::Vector2d> > gPixelClusters;
std::vector<Eigen::Vector2d> gClusterCentroids;
std::vector<double> gTableCoeffs;
pcl::PointCloud<PointTa> gTablePoints;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void process( int state, void* userdata );
void save( int state, void* userdata );
void noDrawSegment( int state, void* userdata );

void drawSegmented();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  gCounter = 0;

  // Initialization
  srand( time(NULL) );
  
  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 );
  
  if( !capture.isOpened() ) {
    printf( "\t * Could not open the capture object \n" );
    return -1;
  }
  
  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on
  
  // Set control panel
  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
  
  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Save", save, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );  
  cv::createButton( "No draw", noDrawSegment, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  // Set mouse callback 
  cv::setMouseCallback( gWindowName, onMouse, 0 );

  // Focal distance
  gF = (float)capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );
 
  // Open a file to store info

  // Loop
  for(;;) {
    
    if( !capture.grab() ) {
      printf( "\t * [ERROR] Could not grab a frame \n" );
      return -1;
    }
    
    capture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    if( gIsSegmentedFlag ) {
      drawSegmented();
    }
    cv::imshow( gWindowName, gRgbImg );
    
    capture.retrieve( gPclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }

  } // end for
 
  gOutput.close();
 
  return 0;
}

/**
 * @function noDrawSegment
 * @brief Do not draw the segmentated objects in color (to get the image)
 */
void noDrawSegment( int state, void* userdata ) {
  gIsSegmentedFlag = false;
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
  cv::Point3f p; Eigen::Vector3d currentPoint;
  p = gPclMap.at<cv::Point3f>(y,x);
  currentPoint << (double)-1*p.x, (double)p.y, (double)p.z;

  // Check what segmented object is selected
  if( gClusterCentroids.size() > 0 ) {
    Eigen::Vector2d p; p << (double)x, (double) y;
    double dist;
    double minDist = 1000; int minInd = -1;
    for( int i = 0; i < gClusterCentroids.size(); ++i ) {
      dist = ( p - gClusterCentroids[i] ).norm();
      if( dist < minDist ) {
	minDist = dist; minInd = i; 
      }
    }    
    gSelectedSegmentedCloud = minInd;

  } // end if
}



/**
 * @function save
 */
void save( int state, void* userData ) {
  
  if( gSelectedSegmentedCloud < 0 || gSelectedSegmentedCloud >= gClusters.size() ) {
    printf("--> ERROR: Did not select a cloud? \n");
    return;
  }
   
  // 3. Save segmented cloud + table normal
  char name[50];
  sprintf(name, "cloud_%d.pcd", gCounter ); 
  pcl::io::savePCDFile( name, gClusters[gSelectedSegmentedCloud], true );
  gOutput << name << " " << gTableCoeffs[0] << " " << gTableCoeffs[1] << " " 
	<< gTableCoeffs[2] << " " << gTableCoeffs[3] << std::endl; 
  gCounter++;
}


/**
 * @function process
 */
void process( int state, 
	      void* userdata ) {

  // Get organized pointcloud
  pcl::PointCloud<PointTa>::Ptr cloud( new pcl::PointCloud<PointTa> );
  cv::Vec3i col;
  PointTa P;

  int width = gPclMap.cols;
  int height = gPclMap.rows;

  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false; // some NaN can be found

  cv::Vec3b* ci = gRgbImg.ptr<cv::Vec3b>(0);
  cv::Point3f* pi = gPclMap.ptr<cv::Point3f>(0);
  
  for( size_t j = 0; j < height; ++j ) {   
    for( size_t i = 0; i < width; ++i ) {
      
      P.x = -(*pi).x; P.y = (*pi).y; P.z = (*pi).z;
      P.r = (uint8_t)(*ci)[2];
      P.g = (uint8_t)(*ci)[1]; 
      P.b = (uint8_t)(*ci)[0];
      P.a = 255;
      cloud->points.push_back( P );
      pi++; ci++;
    }
  }
  
  
  // Segment
  TabletopSegmentor<PointTa> tts;
  tts.set_filter_minMax( -0.75, 0.75, -0.75, 0.75, 0.25, 0.9 );
  tts.processCloud( cloud );
  gTableCoeffs = tts.getTableCoeffs();
  gTablePoints = tts.getTable();
  int n = tts.getNumClusters();

  // Set segmented variables
  gIsSegmentedFlag = true;
  gClusters.resize(n);
  gColors.resize(n);
  
  for( int i = 0; i < n; ++i ) {
    gClusters[i] = tts.getCluster(i);
    
    cv::Vec3b def;
    def(0) = rand() % 255; def(1) = rand() % 255; def(2) = rand() % 255;  
    gColors[i] = def;
  }
  getPixelClusters();
  
}


/**
 * @function drawSegmented
 */
void drawSegmented() {
  for( int i = 0; i < gPixelClusters.size(); ++i ) {
    for( int j = 0; j < gPixelClusters[i].size(); ++j ) {
      gRgbImg.at<cv::Vec3b>( gPixelClusters[i][j](1), gPixelClusters[i][j](0) ) = gColors[i];
    }
  }
  
}

/**
 * @function getPixelClusters
 */
void getPixelClusters() {

  gPixelClusters.resize( gClusters.size() );
  gClusterCentroids.resize( gClusters.size() );
  
  for( int i = 0; i < gPixelClusters.size(); ++i ) {
    gPixelClusters[i].resize(0);
  }
  
  int u, v;
  int width, height;
  double X, Y, Z; 
  int sum_u; int sum_v;

  // Get (u,v) pixel of clusters  
  width = gRgbImg.cols;
  height = gRgbImg.rows;

  for( int i = 0; i < gClusters.size(); ++i ) {
   
    sum_u = 0;
    sum_v = 0;

    for( pcl::PointCloud<PointTa>::iterator it = gClusters[i].begin();
	 it != gClusters[i].end(); ++it ) {

      X = (*it).x; Y = (*it).y; Z = (*it).z;     
      u = width/2 - (int)(X*gF/Z);
      v = height/2 -(int)(Y*gF/Z);
      
      gPixelClusters[i].push_back( Eigen::Vector2d(u,v) );

      sum_u += u;
      sum_v += v;
    }
    
    Eigen::Vector2d ct;
    ct << (double)(sum_u)/gClusters[i].points.size(), (double)(sum_v)/gClusters[i].points.size();
    gClusterCentroids[i] = ct;

  }
  
}
