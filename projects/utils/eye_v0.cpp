/**
 * @file eye_v0
 * @brief Captures a snapshot, segments and select an object to be picked up
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
#include "perception/msgs/perception_msgs.h"


/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string windowName = std::string("Robot View");
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

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void startComm( int state, void* userdata );
void process( int state, void* userdata );
void sendMsg( int state, void* userdata );

void drawSegmented();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
printf("Main \n");
  // Initialization
  srand( time(NULL) );
printf("Open NI 2 \n");
  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 );
  
  if( !capture.isOpened() ) {
    std::cout << "\t * Could not open the capture object"<<std::endl;
    return -1;
  } printf("Return panel \n");
  // Set control panel
  cv::namedWindow( windowName, cv::WINDOW_AUTOSIZE );

  
  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  int value;
  cv::createTrackbar("track1", "Robot view", &value, 255, NULL, NULL );

  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );


  cv::createButton( "Send", sendMsg, 
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
 * @function onMouse
 * @brief Stores the current position of clicked point and calculates the world position from the robot's kinematics
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
 * @function startComm
 */
void startComm( int state, void* userdata ) {

  printf("\t * Start communication \n");
  sns_init();
  sns_start();

  sns_chan_open( &segmented_cloud_chan, "segmented-cloud", NULL );  
  printf("\t * [OK] Communication stablished and ready to go \n");

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

   pcl::io::savePCDFileASCII ("icra_pcd.pcd", *cloud);

  // Segment
  TabletopSegmentor<pcl::PointXYZRGBA> tts;

  tts.processCloud( cloud );
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
 * @function process
 */
void sendMsg( int state, void* userdata ) {

  // Send through channel
  uint32_t n_points = 0;
  for( int i = 0; i < clusters.size(); ++i ) { n_points += clusters[i].size(); }
  printf("Number of points: %d \n", n_points );
  struct sns_msg_segmented_cloud* msg = sns_msg_segmented_cloud_heap_alloc( n_points );
  
  sns_msg_segmented_cloud_init( msg, n_points );
  msg->n_clusters = clusters.size();
  msg->selected = selectedSegmentedCloud;


  int count = 0;
  
  for( int i = 0; i < clusters.size(); ++i ) {
    for( int j = 0; j < clusters[i].size(); ++j ) {
      msg->u[count].x = clusters[i].points[j].x;
      msg->u[count].y = clusters[i].points[j].y;
      msg->u[count].z = clusters[i].points[j].z;
      msg->u[count].cluster = i;
      count++;
    }
  }
  

  ach_status r;
  printf("\t * Size of msg: %d \n", sns_msg_segmented_cloud_size(msg) );
  r = ach_put( &segmented_cloud_chan, msg,sns_msg_segmented_cloud_size(msg) );
  if( r != ACH_OK ) {
    printf("\t * [BAD] Error sending message. Probably OVERFLOW? Increase frame size? \n");

  } else {
    printf("\t * [GOOD] Message sent all right! \n");
  }

 /////////// DEBUG/////////////////
 for( int i = 0; i < clusters.size(); ++i ) {
  char debugName[50];
  sprintf( debugName,"debug_segmented%d.pcd", i );
  pcl::io::savePCDFileASCII (debugName, clusters[i]);
  }
 //////////////DEBUG//////////////

  aa_mem_region_local_release();

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
