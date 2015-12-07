/**
 * @file get_training_data_OR_single_object
 * @brief Based on perceptionPick. Segments objects on a tabletop and store their cropped images for training a SVM/CNN
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
#include "tabletop_segmentation/tabletop_segmentation.h"

#include <stdlib.h>
#include <time.h>

typedef pcl::PointXYZRGBA PointTa;

/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string gWindowName = std::string("Get Training Data ObjRec Simple Object");
size_t gSaveCalls = 0;
cv::Mat gRgbImg;
cv::Mat gPclMap;
int gPad;
cv::VideoCapture gCapture;

Eigen::Vector3d gCurrentPoint;
bool gIsSegmentedFlag = false;
double gF;

int gSelectedSegmentedCloud = -1;
std::vector<pcl::PointCloud<PointTa> > gClusters;
std::vector<cv::Vec3b> gColors;
std::vector< std::vector<Eigen::Vector2d> > gPixelClusters;
std::vector<Eigen::Vector2d> gClusterCentroids;
std::vector<Eigen::Vector4d> gBoundingBoxes;
std::vector<double> gTableCoeffs;
pcl::PointCloud<PointTa> gTablePoints;
std::string gName;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void process( int state, void* userdata );
void save( int state, void* userdata );
void drawSegmented();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Store name of batch
  if( argc >= 2 ) {
    gName = std::string( argv[1] );
    printf("Object name: %s \n", gName.c_str() );
    if( argc >= 3 ) {
      gPad = atoi(argv[2] ); printf("Pad offset: %d \n", gPad);
    } else{ gPad = 0; }
  } else {
    gName = std::string( "default" );
  }

  // Initialization
  srand( time(NULL) );  
  gCapture.open( cv::CAP_OPENNI2 );
  
  if( !gCapture.isOpened() ) {
    printf( "\t [ERROR] Could not open the capture object \n" );
    return -1;
  }

  printf("\t [INFO] Opened device \n");
  
  gCapture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  gCapture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on
  gF = (float)gCapture.get( cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );
  
  printf("\t [INFO] Image dimensions: (%f,%f) \n",
	 gCapture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 gCapture.get( cv::CAP_PROP_FRAME_HEIGHT ) );
  

  // Set control panel
  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
  
  int value;
  cv::createTrackbar("track1", gWindowName.c_str(), &value, 255, NULL, NULL );
  
  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Save crops", save, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  
  // Set mouse callback 
  cv::setMouseCallback( gWindowName, onMouse, 0 );

  // Loop
  for(;;) {
    
    if( !gCapture.grab() ) {
      printf( "\t [ERROR] Could not grab a frame \n" );
      return -1;
    }
    
    gCapture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    if( gIsSegmentedFlag ) {
      drawSegmented();
    }
    cv::imshow( gWindowName, gRgbImg );
    
    gCapture.retrieve( gPclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    

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

  std::cout << "\t [INFO] Current point: "<< currentPoint.transpose() << std::endl;

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

    printf( "\t [INFO] Segmented cloud to send: %d \n", gSelectedSegmentedCloud );
  } // end if
}

/**
 * @function save
 * @brief Save segmented bounding boxes around objects
 */
void save( int state, void* userdata ) {

  // Don't draw
  gCapture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
  gCapture.retrieve( gPclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );

  int i = gSelectedSegmentedCloud;  // In this case, only one pointcloud
  // Create subimage from the bounding box

  int xl = gBoundingBoxes[i](0);
  int yl = gBoundingBoxes[i](1);
  int xw = gBoundingBoxes[i](2)-gBoundingBoxes[i](0);
  int yw = gBoundingBoxes[i](3)-gBoundingBoxes[i](1);

  int mWidth = gRgbImg.cols;
  int mHeight = gRgbImg.rows;

  if( xl - gPad >= 0 ) { xl -= gPad; }
  if( xl + xw + 2*gPad < mWidth ) { xw += 2*gPad; }
  else if( xl + xw + gPad < mWidth ) { xw += gPad; }
 
  if( yl - gPad >= 0 ) { yl -= gPad; }
  if( yl + yw + 2*gPad < mHeight ) { yw += 2*gPad; }
  else if( yl + yw + gPad < mHeight ) { yw += gPad; }


  cv::Mat img( gRgbImg, cv::Rect( xl, yl,
				  xw, yw ) );

  cv::Mat img_depth( gPclMap, cv::Rect( xl, yl,
				  xw, yw ) );


  // Store
  char imageName[100];
  sprintf( imageName, "%s_cropped_rgb_%ld.png", gName.c_str(), gSaveCalls );
  cv::imwrite( imageName, img );
  sprintf( imageName, "%s_original_rbg_%ld.png", gName.c_str(), gSaveCalls );
  cv::imwrite( imageName, gRgbImg );
  sprintf( imageName, "%s_cropped_depth_%ld.yml", gName.c_str(), gSaveCalls );
  cv::FileStorage fs( imageName, cv::FileStorage::WRITE );
  fs << "depthMatrix" << img_depth;
  fs.release();
  pcl::io::savePCDFile( "selectedCloud.pcd", gClusters[gSelectedSegmentedCloud], true );  
  printf("Saved %s \n", imageName );

  gSaveCalls++;
}

/**
 * @function process
 */
void process( int state, void* userdata ) {


    // Get a clear image (original color)
    gCapture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    gCapture.retrieve( gPclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );

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
  tts.set_filter_minMax( -0.85, 0.85, -0.85, 0.85, 0.25, 1.0 );
  tts.processCloud( cloud );
  gTableCoeffs = tts.getTableCoeffs();
  gTablePoints = tts.getTable();
  int n = tts.getNumClusters();

  // Set segmented variables
  gIsSegmentedFlag = true;
  gClusters.resize(n);
  gColors.resize(n);

  int maxPoints = 0;

  for( int i = 0; i < n; ++i ) {
    gClusters[i] = tts.getCluster(i);
    
    gColors[i] = cv::Vec3b( rand() % 255, 
			    rand() % 255, 
			    rand() % 255 );

    if( gClusters[i].points.size() > maxPoints ) {
      maxPoints = gClusters[i].points.size();
      gSelectedSegmentedCloud = i;
    } 
  }
  getPixelClusters();
  
}


/**
 * @function drawSegmented
 */
void drawSegmented() {
  for( int i = 0; i < gPixelClusters.size(); ++i ) {

    int thickness = 2;
    cv::rectangle( gRgbImg, 
		   cv::Point( gBoundingBoxes[i](0), gBoundingBoxes[i](1) ),
		   cv::Point( gBoundingBoxes[i](2), gBoundingBoxes[i](3) ),
		   gColors[i], thickness );
  }
  
}

/**
 * @function getPixelClusters
 */
void getPixelClusters() {

  gPixelClusters.resize( gClusters.size() );
  gClusterCentroids.resize( gClusters.size() );
  gBoundingBoxes.resize( gClusters.size() );

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

    int min_u = width; int min_v = height;
    int max_u = 0; int max_v = 0;

    for( pcl::PointCloud<PointTa>::iterator it = gClusters[i].begin();
	 it != gClusters[i].end(); ++it ) {

      X = (*it).x; Y = (*it).y; Z = (*it).z;     
      u = width/2 - (int)(X*gF/Z);
      v = height/2 -(int)(Y*gF/Z);
      
      gPixelClusters[i].push_back( Eigen::Vector2d(u,v) );

      sum_u += u;
      sum_v += v;

      // Bounding Box
      if( u < min_u ) { min_u = u; }
      if( u > max_u ) { max_u = u; }
      if( v < min_v ) { min_v = v; }
      if( v > max_v ) { max_v = v; }

    }

    Eigen::Vector4d mM; mM << min_u, min_v, max_u, max_v;
    gBoundingBoxes[i] = mM;

    Eigen::Vector2d ct;
    ct << (double)(sum_u)/gClusters[i].points.size(), (double)(sum_v)/gClusters[i].points.size();
    gClusterCentroids[i] = ct;

  }

  
}
