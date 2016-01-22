/**
 * @file ffts_get_object_data_4.cpp
 * @brief Get training data for object recognition
 * @brief Adding bounding box
 * @date 2016/01/13
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>

#include "perception/ftts/fast_tabletop_segmentation.h"
#include <mutex>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

// Global variables
std::string gWindowName = std::string("Fast pick");
cv::Mat gRgbImg; cv::Mat gXyzImg;
pcl::io::OpenNI2Grabber* gGrabber = NULL;
Fast_Tabletop_Segmentation<PointT> gTts;
std::mutex  mutex;

// Bounding box
std::vector<Eigen::VectorXd> gClustersBB;

// Storing
bool gShowSegmentation = true;
std::string gName("default");
int gCounter = 0;
int gSelectedCluster = 0;

// Functions
static void onMouse( int event, int x, int y, int flags, void* userdata );
void grabber_callback( const CloudConstPtr& _cloud );
void drawBoundingBox();
void saveData();


//////////////////////////
//  @function main
/////////////////////////
int main( int argc, char* argv[] ) {

  int c;
  while( (c=getopt(argc,argv,"n:")) != -1 ) {
    switch (c) {
    case 'n': {gName = std::string(optarg); } break;
    }
  }

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
 * @function saveData
 * @brief Save rgb and depth matrix + pointcloud + rgb with different overlay colors on the object
 */
void saveData() {

  int i = gSelectedCluster;
  const int num_overlays = 7;
  uchar r[num_overlays] = {255,255,0,0,255,255,0};
  uchar g[num_overlays] = {255,0,255,0,255,0,255};
  uchar b[num_overlays] = {255,0,0,255,0,255,255};

  // Save rgb sub-img
  cv::Mat img( gRgbImg, cv::Rect(gClustersBB[i](0), gClustersBB[i](1),
				 gClustersBB[i](2) - gClustersBB[i](0),
				 gClustersBB[i](3) - gClustersBB[i](1) ) );
  char name[50];
  sprintf( name, "%s_rgb_%d.png",gName.c_str(),  gCounter );   
  cv::imwrite( name, img );
  
  // Save depth sub-img
  cv::Mat dimg( gXyzImg, cv::Rect(gClustersBB[i](0), gClustersBB[i](1),
				  gClustersBB[i](2) - gClustersBB[i](0),
				  gClustersBB[i](3) - gClustersBB[i](1) ) );  
  sprintf( name, "%s_xyz_%d.yml", gName.c_str(), gCounter );
  cv::FileStorage fs( name, cv::FileStorage::WRITE );
  fs << "xyzMatrix" << dimg; fs.release();
  // Save pointcloud
  sprintf( name, "%s_%d.pcd", gName.c_str(), gCounter );

  pcl::io::savePCDFileASCII(name, *gTts.getCluster(i));
  // Optionally, save rgb img with different overlays

  pcl::PointIndices ind = gTts.getClusterIndices(i);
  cv::Mat m = gRgbImg.clone();
  for( int j = 0; j < num_overlays; ++j ) {
    // Overlay color
    for( int k = 0; k < ind.indices.size(); ++k ) {
      int row, col;
      row = ind.indices[k] / 640;
      col = ind.indices[k] % 640;
      uchar ri, gi, bi;
      ri = (uchar)( ( gRgbImg.at<cv::Vec3b>(row,col)(2) + r[j] ) / 2 );
      gi = (uchar)( ( gRgbImg.at<cv::Vec3b>(row,col)(1) + g[j] ) / 2 );
      bi = (uchar)( ( gRgbImg.at<cv::Vec3b>(row,col)(0) + b[j] ) / 2 );

      m.at<cv::Vec3b>(row, col) = cv::Vec3b(bi,gi,ri);
    }
    sprintf( name, "%s_rgb_overlay_%d_%d.png", gName.c_str(), j, gCounter );
    
    cv::imwrite( name, cv::Mat(m,cv::Rect(gClustersBB[i](0),
					  gClustersBB[i](1),
					  gClustersBB[i](2) - gClustersBB[i](0),
					  gClustersBB[i](3) - gClustersBB[i](1) )) );
  }

  printf("Saved %s counter %d \n", gName.c_str(), gCounter );

  // Augment the counter
  gCounter++;
}

/**
 * @function onMouse
 * @brief Tells the point location
 */
static void onMouse( int event, int x, int y, int flags, void* userdata ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }
  mutex.lock();
  cv::Point3f p; Eigen::Vector3d currentPoint;
  p = gXyzImg.at<cv::Point3f>(y,x);
  currentPoint << (double)p.x, (double)p.y, (double)p.z;
  std::cout << "\t * Mouse pressed. Current point: "<< currentPoint.transpose() << std::endl;
  
  double minDist; int minIndex; double dist;

  gSelectedCluster = -1;

    for( int i = 0; i < gClustersBB.size(); ++i ) {
      int cx, cy;
      cx = (gClustersBB[i](0) + gClustersBB[i](2) )/2;
      cy = (gClustersBB[i](1) + gClustersBB[i](3) )/2;
      dist = (x-cx)*(x-cx) + (y-cy)*(y-cy);
      if( i == 0 ) { minIndex = 0; minDist = dist; }
      else { if( dist < minDist ) { minIndex = i; minDist = dist; } }
    }

    gSelectedCluster = minIndex;

    saveData();
    mutex.unlock();
}

/**
 * @function grabber_callback
 */
void grabber_callback( const CloudConstPtr& _cloud ) {
  mutex.lock();
  double dt; clock_t ts, tf;
  // Segment the new input
  gTts.process( _cloud, gShowSegmentation );
  // Show it
  gRgbImg = gTts.getRgbImg();
  gXyzImg = gTts.getXyzImg();
  // Draw bounding box
  drawBoundingBox();
  mutex.unlock();
}

/**
 * @function drawingBoundingBox
 * @brief BB is a bit larger than it should (10 pixels)
 */
void drawBoundingBox() {

  int xmin, ymin, xmax, ymax;

  cv::Vec3b colors; colors(0) = 255; colors(1) = 0; colors(2) = 0;
  gClustersBB.resize(0);

  for( int i = 0; i < gTts.getNumClusters(); ++i ) {
    gTts.getClusterBB( i, xmin, ymin, xmax, ymax );
    // Draw a bit bigger
    int dx = 10; int dy = 10;
    if( xmin - dx > 0 ) { xmin = xmin - dx; }
    if( xmax + dx < 640 - 1 ) { xmax = xmax + dx; }
    if( ymin -dy > 0 ) { ymin = ymin - dy; }
    if( ymax + dy < 480 - 1 ) { ymax = ymax + dy; }
    cv::rectangle( gRgbImg, cv::Point( xmin, ymin), cv::Point(xmax, ymax), colors, 1 );    
    gClustersBB.push_back( Eigen::Vector4d(xmin+1,ymin+1,xmax-1,ymax-1) );
  }

}
