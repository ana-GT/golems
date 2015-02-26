/**
 * @file getSegmentedClouds
 * @brief Captures a snapshot, segments and store with given name
 * @author A. Huaman Q.
 */
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <stdint.h>
#include "tabletop_segmentation/tabletop_segmentation.h"
#include "msgs/perception_msgs.h"


/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string windowName = std::string("Robot View");
cv::Mat rgbImg;
cv::Mat labeledRgbImg;
cv::Mat pclMap;

ach_channel_t segmented_cloud_chan;
Eigen::Vector3d currentPoint;
bool isSegmentedFlag = false;
double f;

int selectedSegmentedCloud = -1;
std::vector<pcl::PointCloud<pcl::PointXYZRGB> > clusters;
std::vector<cv::Vec3b> colors;
std::vector< std::vector<Eigen::Vector2d> > pixelClusters;
std::vector<Eigen::Vector2d> clusterCentroids;
std::string filename;
int counter;
std::vector<double> tableCoeffs(4);

TabletopSegmentor<pcl::PointXYZRGB> tts;

/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void process();
void printHelp( char* argv0 );


void drawSegmented();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  int c;
  filename = std::string( "default" );
  counter = 0;
  while( (c=getopt(argc, argv, "hn:")) != -1 ) {
    switch(c) {
    case 'h' : { printHelp(argv[0] ); return 1; } break;
    case 'n': { filename = std::string( optarg ); } break;
    }
  }


  // Initialization
  srand( time(NULL) );

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI );
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }
  
  // Set control panel
  cv::namedWindow( windowName, cv::WINDOW_AUTOSIZE );

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
    rgbImg.copyTo( labeledRgbImg );
    
    if( isSegmentedFlag ) {
      drawSegmented();
    }
    cv::imshow( windowName, labeledRgbImg );
         
    capture.retrieve( pclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }

    if( k == 's' ) {
      printf("Storing %s_%d... \n", filename.c_str(), counter );
      process();
      printf("Done storing\n");
    }
    
    if( k == 'p' ) {
      printf("Storing rgb %s_%d... \n", filename.c_str(), counter );
      char name[50];
      sprintf( name, "%s_%d.png", filename.c_str(), counter );
      cv::imwrite( name, rgbImg );
      printf("Done storing\n");
    }


  } // end for
  
  return 0;
}

/**
 * @function onMouse
 * @brief Stores the current position of clicked point and stores the corresponding pointcloud
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

    char name[50];
    sprintf(name, "%s_%d.pcd", filename.c_str(), counter );
    pcl::io::savePCDFileASCII ( name, clusters[selectedSegmentedCloud] );


    char tableName[50];
    pcl::PointCloud<pcl::PointXYZRGB> tablePts;
    tablePts = tts.getTable();
    sprintf(tableName, "%s_%d_table.pcd", filename.c_str(), counter );
    pcl::io::savePCDFileASCII ( tableName, tablePts );


     // Save table coeffs
     FILE* pFile;
     char tableCoeffName[50];
     sprintf(tableCoeffName, "%s_%d.txt", filename.c_str(), counter );
     pFile = fopen( tableCoeffName, "w" );
     fprintf( pFile, " %f %f %f %f \n", tableCoeffs[0], tableCoeffs[1], tableCoeffs[2], tableCoeffs[3] );
     fclose( pFile );


     std::cout << "\t [INFO] Segmented cloud stored: "<< name << std::endl;
     counter++;
   } // end if
 }


 /**
  * @function process
  * @brief Segments the cluster pointclouds
  */
 void process() {

   // Get organized pointcloud
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
   cv::Point3f p;
   cv::Vec3i col;
   pcl::PointXYZRGB P;

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
       cloud->points[width*j + i] = P;

     }
   }



   // Segment
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

   tableCoeffs = tts.getTableCoeffs();

 }


 /**
  * @function drawSegmented
  */
 void drawSegmented() {
   for( int i = 0; i < pixelClusters.size(); ++i ) {
     for( int j = 0; j < pixelClusters[i].size(); ++j ) {
       labeledRgbImg.at<cv::Vec3b>( pixelClusters[i][j](1), pixelClusters[i][j](0) ) = colors[i];
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



/**
 * @function printHelp
 */
void printHelp( char* argv0 ) {
  printf("Usage: \n");
  printf("\t -h: This help \n");
  printf("\t -n FILENAME: Name of this pointcloud  \n");

}
