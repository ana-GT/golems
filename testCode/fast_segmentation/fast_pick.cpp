

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


std::string gWindowName = std::string("Fast pick");
cv::Mat gRgbImg;
pcl::io::OpenNI2Grabber* gGrabber = NULL;
Fast_Tabletop_Segmentation<PointT> gTts;

/**
 * @function grabber_callback
 */
void grabber_callback( const CloudConstPtr& _cloud ) {

  // Store pointcloud in image
  int data_size_ = _cloud->width * _cloud->height*3;
  unsigned char* data_ = new unsigned char[data_size_];

  int j = 0;
  for( size_t i = 0; i < _cloud->points.size(); ++i ) {
    data_[j++] = _cloud->points[i].b;
    data_[j++] = _cloud->points[i].g;
    data_[j++] = _cloud->points[i].r;
  }

  // Store RGB image
  gRgbImg = cv::Mat( _cloud->height, _cloud->width, CV_8UC3, data_ );

  // Segment the new input
  gTts.compute( _cloud );
}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  gRgbImg = cv::Mat( 480, 640, CV_8UC3 );

  // Set capture
  gGrabber = new pcl::io::OpenNI2Grabber("", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode, 
					 pcl::io::OpenNI2Grabber::OpenNI_Default_Mode );
  boost::function<void (const CloudConstPtr&) > f =
    boost::bind(&grabber_callback, _1 );
  gGrabber->registerCallback(f);

  //Loop
  gGrabber->start();

  for(;;) {
    
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [ESC] Finishing program! \n");
      gGrabber->stop();
      break;
    }

  cv::imshow( gWindowName, gRgbImg );

  } // end for

  return 0;

}
