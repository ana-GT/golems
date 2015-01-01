/**
 * @file grab_data.cpp
 * @brief Grab RGB  and pointcloud corresponding
 * @author A. Huaman Q.
 */

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

cv::Mat rgbImg;
cv::Mat pclMap;
std::string mWindowName( "Crichton View" );

int counter = 0;

/***********************/
/** FUNCTIONS          */
/***********************/
void saveData( int state, void* userdata );
static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p;
  p = pclMap.at<cv::Point3f>(y,x);
  printf("Depth: %f %f %f \n", p.x, p.y, p.z);
 
}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 ); 
  
  if( !capture.isOpened() ) {
    std::cout << "/t * Could not open the capture object"<<std::endl;
    return -1;
  }
  printf("\t * Opened device \n");

  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 1.0 );
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on

  printf("\t * Common Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_PROP_FRAME_HEIGHT ) );

  printf("\t * Only Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ) );

  printf("\t * Only Depth dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT ) );

  
  // Set control panel
  int value;
  cv::namedWindow( mWindowName );
  // Set mouse callback 
  cv::setMouseCallback( mWindowName, onMouse, 0 );
  cv::createButton( "Save", saveData, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  // Loop
  for(;;) {
    if( !capture.grab() ) {
      std::cout << "\t * [DAMN] Could not grab a frame" << std::endl;
      return -1;
    }

    capture.retrieve( rgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    cv::imshow( mWindowName, rgbImg );

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
 * @function process
 */
void saveData( int state, void* userdata ) {

  char name_rgb[50];
  char name_pcl[50];
  sprintf(name_rgb, "image_%d.png", counter );
  sprintf(name_pcl, "pointcloud_%d.pcd", counter );
    
  cv::imwrite( name_rgb, rgbImg );
  printf("\t * Stored image %s \n", name_rgb);
  
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
      P.x = p.x; P.y = p.y; P.z = p.z;
      P.r = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[2];
      P.g = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[1]; 
      P.b = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[0];
      P.a = 255;

      if( p.z != p.z ) {
	P.r = 255;
	P.g = 0; 
	P.b = 255;
      }
      
      cloud->points[width*j + i] = P;
      
    }
  }
   pcl::io::savePCDFileASCII (name_pcl, *cloud);

  printf("\t * Stored pcl %s \n", name_pcl);
  
  counter++;
}

