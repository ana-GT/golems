
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <stdint.h>  
#include "tabletop_segmentation/tabletop_segmentation.h"

#include <caffe/caffe.hpp>
#include <sstream>


#include "classifier.h"

typedef pcl::PointXYZRGBA PointTa;

std::string gWindowName("recognition_1");
cv::VideoCapture gCapture;
cv::Mat gRgbImg;
cv::Mat gPclMap;
Eigen::Vector3d gCurrentPoint;
bool gIsSegmentedFlag = false;
double gF; 

std::vector<pcl::PointCloud<PointTa> > gClusters;
std::vector<cv::Vec3b> gColors;
std::vector< std::vector<Eigen::Vector2d> > gPixelClusters;
std::vector<Eigen::Vector2d> gClusterCentroids;
std::vector<Eigen::Vector4d> gBoundingBoxes;
std::vector<double> gTableCoeffs;
pcl::PointCloud<PointTa> gTablePoints;
std::vector<std::string> gLabels;
std::vector<int> gIndex;
std::vector<std::string> gHabla;

char* gModel_file ="/home/ana/Desktop/Crichton_data_trained/deploy_alexnet.prototxt";
char* gTrain_file = "/home/ana/Desktop/Crichton_data_trained/partial_alexnet_iter_1700.caffemodel";
// Remember AlexNet, referenc_caffenet and RCNN_ilsvrc13: 227, googlenet: 224
char* gMean_file = "/home/ana/Desktop/Crichton_data_processed/Crichton_data_227_brute_resize_train.binaryproto";
char* gLabel_file = "/home/ana/Desktop/Crichton_data/training_labels.txt";
char* gExplanations= "/home/ana/Desktop/Crichton_data/training_labels_explanatory.txt";
int gBiggestCluster;

/*** Functions */
void process();
void drawSegmented();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  // http://www.robots.ox.ac.uk/~vgg/research/very_deep/  
  int c;
  while( (c=getopt(argc,argv,"n:t:m:l:h")) != -1 ) {
    switch(c) {
    case 'n': { gModel_file = optarg; } break;
    case 't': { gTrain_file = optarg; } break;
    case 'm': { gMean_file = optarg; } break;
    case 'l': { gLabel_file = optarg; } break;
    case 'h': { printf("Syntax: %s -n deploy.txt -t trained.caffemodel -m mean.binaryproto -l labels.txt \n",
		       argv[0] ); return 1; } break;
    }
  }

  Classifier gClassifier( gModel_file, gTrain_file, gMean_file, gLabel_file );

   ////////////////////////////////  
  // Load explanations
  printf("Load explanations \n");
  std::ifstream  explan( gExplanations, std::ifstream::in );
  std::string line;
  std::string words;
  while( std::getline(explan, line) ) {
    std::size_t pos = line.find("Veo");
    words = line.substr(pos);
    printf("Line: %s\n", words.c_str() );
    gHabla.push_back(words);
  }
  explan.close();
  printf("Finished loading explanations\n");
  ////////////////////////////////

  gCapture.open( cv::CAP_OPENNI2 );
  
  if( !gCapture.isOpened() ) {
    printf("\t [ERROR] Could not open the capture object \n");
    return -1;
  }

  gCapture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 );
  gCapture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 );
  gF = (float)gCapture.get( cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );

  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
 
  ::google::InitGoogleLogging( argv[0] );
  
  for(;;) {

    if( !gCapture.grab() ) {
      printf("\t * ERROR Could not grab a frame \n");
      return -1;
    }

    gCapture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    if( gIsSegmentedFlag ) { drawSegmented(); }
    cv::imshow( gWindowName, gRgbImg );
    
    gCapture.retrieve( gPclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );


    cv::imshow( gWindowName, gRgbImg );

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t Finishing program \n");
      break;
    } 

    /** Recognize */
    else if( k == 'i' ) {

      // Process image
      process();
      gLabels.resize(gClusters.size() );
      gIndex.resize(gClusters.size() );      
      // Store images
      for( int i = 0; i < gClusters.size(); ++i ) {

	int xl = gBoundingBoxes[i](0);
	int yl = gBoundingBoxes[i](1);
	int xw = gBoundingBoxes[i](2)-gBoundingBoxes[i](0);
	int yw = gBoundingBoxes[i](3)-gBoundingBoxes[i](1);
	/*
	  int mWidth = gRgbImg.cols;
	  int mHeight = gRgbImg.rows;
	  
	  if( xl - gPad >= 0 ) { xl -= gPad; }
	  if( xl + xw + 2*gPad < mWidth ) { xw += 2*gPad; }
	  else if( xl + xw + gPad < mWidth ) { xw += gPad; }
	  
	  if( yl - gPad >= 0 ) { yl -= gPad; }
	  if( yl + yw + 2*gPad < mHeight ) { yw += 2*gPad; }
	  else if( yl + yw + gPad < mHeight ) { yw += gPad; }
	*/
	
	cv::Mat img( gRgbImg, cv::Rect( xl, yl,
					xw, yw ) );
	
	printf("xl: %d yl: %d xw: %d zw: %d \n", xl, yl, xw, yw);
	printf("Segmented image [%d] size: %d, %d \n", i, img.rows, img.cols );
	// Predict 
	int idx;
	std::vector<Prediction> predictions = gClassifier.classify( img, idx );
	gLabels[i] = predictions[0].first;
        gIndex[i] = idx;

    cv::putText( gRgbImg,
		 gLabels[i], cv::Point(gBoundingBoxes[i](0), gBoundingBoxes[i](1) ),
		 cv::FONT_HERSHEY_SIMPLEX, 1, 
		 gColors[i],
		 2 );


    char texto[150];
    sprintf(texto, "espeak '%s' -p 80 -s 200 -ves-la ", gHabla[gIndex[i]].c_str() );
    printf("Texto: %s \n", texto);
    system(texto); usleep(0.1*1e6);
     printf("Finished speaking\n");
	
      }
      
      
      
    } // else
    
    
  } // for
  
} // main

/**
 * @function process
 */
void process() {

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
  tts.set_min_cluster_size(1500);
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
    
    gColors[i] = cv::Vec3b( rand() % 255, 
			    rand() % 255, 
			    rand() % 255 );
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

    cv::putText( gRgbImg,
		 gLabels[i], cv::Point(gBoundingBoxes[i](0), gBoundingBoxes[i](1) ),
		 cv::FONT_HERSHEY_SIMPLEX, 1, 
		 gColors[i],
		 2 );
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
    printf("CLuster %d size: %d \n", i, gPixelClusters[i].size());
    Eigen::Vector4d mM; mM << min_u, min_v, max_u, max_v;
    gBoundingBoxes[i] = mM;

    Eigen::Vector2d ct;
    ct << (double)(sum_u)/gClusters[i].points.size(), (double)(sum_v)/gClusters[i].points.size();
    gClusterCentroids[i] = ct;

  }

  
}
