/**
 * @file fast_perception_pick_2.cpp
 * @brief Using component-based segmentation for fast performance (~5Hz)
 * @brief Adding bounding box
 * @date 2016/01/13
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>

#include "fast_tabletop_segmentation.h"

#include "classifier.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

// Global variables
std::string gWindowName = std::string("Fast pick");
cv::Mat gRgbImg; cv::Mat gXyzImg;
pcl::io::OpenNI2Grabber* gGrabber = NULL;
Fast_Tabletop_Segmentation<PointT> gTts;
bool gShowSegmentation = false;

// Bounding box

// Recognition
/*
char* gModel_file ="/home/ana/Software/caffe/models/YCB_svm/deploy.prototxt";
char* gTrain_file = "/home/ana/Software/caffe/models/YCB_svm/partial_YCB_svm_iter_6000.caffemodel";
char* gMean_file = "/home/ana/Software/caffe/models/YCB_svm/mean_YCB.binaryproto";
char* gLabel_file = "/home/ana/Software/caffe/models/YCB_svm/training_labels.txt";
*/
char* gModel_file ="/home/ana/Software/caffe/models/YCB_vgg/deploy.prototxt";
char* gTrain_file = "/home/ana/Software/caffe/models/YCB_vgg/YCB_svm_train_thoughtful_iter_3500.caffemodel";
char* gMean_file = "/home/ana/Software/caffe/models/YCB_vgg/mean_YCB.binaryproto";
char* gLabel_file = "/home/ana/Software/caffe/models/YCB_vgg/training_labels.txt";

std::vector<std::string> gLabels;

// Functions
static void onMouse( int event, int x, int y, int flags, void* userdata );
void grabber_callback( const CloudConstPtr& _cloud );
void drawBoundingBox();

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

  // Classifier
  printf("Init classifier\n");
  Classifier gClassifier( gModel_file, gTrain_file, gMean_file, gLabel_file );
  printf("End init classifier\n");
  for(;;) {
    
    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * Pressed ESC. Finishing program! \n");
      gGrabber->stop();
      break;
    } else if( k == 'r' ) {

      // Get image
      
      // Store images
      int xmin, ymin, xmax, ymax;
      gLabels.resize( gTts.getNumClusters() );
      for( int i = 0; i < gTts.getNumClusters(); ++i ) {
	printf("Get cluster \n");
	gTts.getClusterBB( i, xmin, ymin, xmax, ymax );
	cv::Mat img( gRgbImg, cv::Rect(xmin,ymin,xmax-xmin, ymax-ymin) );
	char name[50];
	sprintf( name, "image_%d.png", i );
	printf("Write img\n");
	cv::imwrite( name, img );
	// Predict
	printf("Predict \n");
	std::vector<Prediction> predictions = gClassifier.classify(img);
	printf("Predicted \n");
	printf("Prediction %d: ", i);
	for( int k = 0; k < predictions.size(); ++k ) {
	  printf(" %d: %s . ", k, predictions[k].first.c_str() );
	} printf("\n");
	gLabels[i] = predictions[0].first;
	printf("Prediction [%d]: %s \n", i, gLabels[i].c_str());
      }
      
    } // else if

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
  gTts.process( _cloud, gShowSegmentation );
  // Show it
  gRgbImg = gTts.getRgbImg();
  gXyzImg = gTts.getXyzImg();
  // Draw bounding box
  drawBoundingBox();
}

void drawBoundingBox() {

  int xmin, ymin, xmax, ymax;

  cv::Vec3b colors; colors(0) = 255; colors(1) = 0; colors(2) = 0;
  for( int i = 0; i < gTts.getNumClusters(); ++i ) {
    gTts.getClusterBB( i, xmin, ymin, xmax, ymax );

    cv::rectangle( gRgbImg, cv::Point( xmin, ymin), cv::Point(xmax, ymax), colors, 2 );
  }

}
