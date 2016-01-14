/**
 * @file fast_perception_pick.cpp
 * @brief Using component-based segmentation for fast performance (~5Hz)
 * @date 2016/01/13
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>

#include "fast_tabletop_segmentation.h"



typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

// Global variables
std::string gWindowName = std::string("Fast pick");
cv::Mat gRgbImg; cv::Mat gXyzImg;
pcl::io::OpenNI2Grabber* gGrabber = NULL;
Fast_Tabletop_Segmentation<PointT> gTts;

// Functions
static void onMouse( int event, int x, int y, int flags, void* userdata );
void grabber_callback( const CloudConstPtr& _cloud );

//////////////////////////
//  @function main
/////////////////////////
int main( int argc, char* argv[] ) {

  gRgbImg = cv::Mat( 480, 640, CV_8UC3 );
  
  // Set capture
  gGrabber = new pcl::io::OpenNI2Grabber("", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode, 
					 pcl::io::OpenNI2Grabber::OpenNI_Default_Mode );
  boost::function<void (const CloudConstPtr&) > f = boost::bind(&grabber_callback, _1 );
  gGrabber->registerCallback(f);

  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
  cv::setMouseCallback( gWindowName, onMouse, 0 );
  
  //Loop
  gGrabber->start();

  for(;;) {
    
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * Pressed ESC. Finishing program! \n");
      gGrabber->stop();
      break;
    }

  cv::imshow( gWindowName, gRgbImg );

  } // end for

  return 0;

}

/**
 * @function onMouse
 * @brief Tells the point location
 */
static void onMouse( int event, int x, int y, int flags, void* userdata ) {

  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }
  
  cv::Point3f p; Eigen::Vector3d currentPoint;
  p = gXyzImg.at<cv::Point3f>(y,x);
  currentPoint << (double)p.x, (double)p.y, (double)p.z;
    std::cout << "\t * Mouse pressed. Current point: "<< currentPoint.transpose() << std::endl;
}

/**
 * @function grabber_callback
 */
void grabber_callback( const CloudConstPtr& _cloud ) {

  double dt; clock_t ts, tf;
  // Segment the new input
  gTts.process( _cloud );
  // Show it
  gRgbImg = gTts.getRgbImg();
  gXyzImg = gTts.getXyzImg();
}
