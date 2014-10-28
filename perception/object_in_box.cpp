/**
 * @file object_in_box
 * @brief Captures a snapshot, segments a box and objects. Send message with one box and selected object only
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
#include "msgs/perception_msgs.h"

#include "box_detector/src/container_detection/include/container_detection/container_detection.h"

/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string windowName = std::string("Robot View");
cv::Mat rgbImg;
cv::Mat pclMap;

ach_channel_t segmented_cloud_chan;
ach_channel_t box_param_chan;
Eigen::Vector3d currentPoint;
bool isSegmentedFlag = false;
bool isBoxDetected = false;
double f;

int selectedSegmentedCloud = -1;
std::vector<pcl::PointCloud<pcl::PointXYZRGB> > clusters;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedBoxCloud( new pcl::PointCloud<pcl::PointXYZRGB> );
pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedObject( new pcl::PointCloud<pcl::PointXYZRGB> );
pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud( new pcl::PointCloud<pcl::PointXYZRGB> );
std::vector<cv::Vec3b> colors;
std::vector< std::vector<Eigen::Vector2d> > pixelClusters;
std::vector<Eigen::Vector2d> clusterCentroids;

//** Box Parameters **//
Eigen::Vector3d boxVertices3D[8];
cv::Point boxVertices2D[8];    
Eigen::Vector4d box_size;
Eigen::Vector3d box_pos;
Eigen::Quaterniond box_rot;




/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void startComm( int state, void* userdata );
void process( int state, void* userdata );

void pickBox( int state, void* userdata );
void pickObject( int state, void* userdata );
void sendBox( int state, void* userdata );
void sendObject( int state, void* userdata );

void drawSegmented();
void drawBox();
void getPixelClusters();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

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

  int value;
  cv::createTrackbar("track1", NULL, &value, 255, NULL, NULL );
  
  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  cv::createTrackbar("track2", NULL, &value, 255, NULL, NULL );

  cv::createButton( "Pick box", pickBox,
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  cv::createButton( "Pick object", pickObject,
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  cv::createTrackbar("track3", NULL, &value, 255, NULL, NULL );

  cv::createButton( "Send box", sendBox,
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  cv::createButton( "Send object", sendObject,
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
    if( isBoxDetected ) {
      drawBox();
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
  sns_chan_open( &box_param_chan, "box", NULL );
  printf("\t * [OK] Communication stablished and ready to go \n");

}


/**
 * @function process
 */
void process( int state, 
	      void* userdata ) {

  // Reset
  isBoxDetected = false;


  // Get organized pointcloud
  cv::Point3f p;
  cv::Vec3i col;
  pcl::PointXYZRGB P;

  int width = pclMap.cols;
  int height = pclMap.rows;

  inputCloud->width = width;
  inputCloud->height = height;
  inputCloud->is_dense = false; // some NaN can be found
  inputCloud->points.resize( width * height );

  for( size_t j = 0; j < height; ++j ) {
    for( size_t i = 0; i < width; ++i ) {

      p = pclMap.at<cv::Point3f>(j,i);
      P.x = p.x*-1; P.y = p.y; P.z = p.z;
      P.r = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[2];
      P.g = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[1]; 
      P.b = (uint8_t)rgbImg.at<cv::Vec3b>(j,i)[0];

      inputCloud->points[width*j + i] = P;
      
    }
  }

  // Segment
  TabletopSegmentor<pcl::PointXYZRGB> tts;

  tts.processCloud( inputCloud );
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
 * @function pickBox
 */
void pickBox( int state, void* userdata ) {

  isBoxDetected = false;

  printf("Picking box \n");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr boxCloud( new pcl::PointCloud<pcl::PointXYZRGB> );

  char dimc[] = { 'x', 'y', 'z'};
  double min_dim[] = {-0.5, -0.5, 0};
  double max_dim[] = {0.5, 0.5, 1.79};
  char argname[3];
  for(size_t i = 0;i < 3; i++)
  {
    printf("Removing all points on %c axis"
	   " below %0.2f m and above %0.2f \n",
	   dimc[i],min_dim[i],max_dim[i]);
    
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud ( inputCloud );
    pass.setKeepOrganized(true);
    sprintf(argname,"%c",dimc[i]);
    pass.setFilterFieldName (argname);
    pass.setFilterLimits (min_dim[i], max_dim[i]); // m
    pass.filter (*boxCloud);
  }


  // Process it
  ContainerDetection<pcl::PointXYZRGB> cdet;
  ContainerDetection<pcl::PointXYZRGB>::params_t params;

  // Parameter tweaking
  params.seg_dist_thres = 0.025;		
  params.seg_ang_thres = 0.105;	  
  params.seg_min_inliers = 800; 
  params.seg_max_curv = 0.05;

  params.est_mode_para=true;
  params.est_para_max_dev = 0.175;
  params.est_para_min_dist = 0.1;
  params.est_perp_max_dev = 0.175;

  cdet.setParams( params );
  cdet.setInputCloud( boxCloud );
  printf("Searching box \n");
  double nboxes = cdet.search();
  printf("Finished searching box \n");
  if( nboxes ) {
    box_pos = cdet.getPosition();
    box_rot = cdet.getRotation();
    box_size = cdet.getSize();
    printf(" * Container pos: %f %f %f \n", box_pos(0),  box_pos(1), box_pos(2) );
    printf(" * Container rot: %f %f %f %f \n", box_rot.x(),  box_rot.y(), 
	   box_rot.z(), box_rot.w() );
    printf(" * Container size: %f %f %f %f\n", box_size(0), box_size(1), box_size(2), box_size(3) );

    // Get the points
    isBoxDetected = true;
    double a, b, c; 
    a = box_size(0); b = box_size(1); c = box_size(2);
    double height, width;
      width = rgbImg.cols;
      height = rgbImg.rows;

    box_rot.normalize();
    Eigen::Matrix3d rot; rot = box_rot.matrix();
    boxVertices3D[0] << a*0.5, b*-0.5, c*0.5;
    boxVertices3D[1] << a*-0.5, b*-0.5, c*0.5;
    boxVertices3D[2] << a*-0.5, b*0.5, c*0.5;
    boxVertices3D[3] << a*0.5, b*0.5, c*0.5;
    boxVertices3D[4] << a*0.5, b*-0.5, c*-0.5;
    boxVertices3D[5] << a*-0.5, b*-0.5, c*-0.5;
    boxVertices3D[6] << a*-0.5, b*0.5, c*-0.5;
    boxVertices3D[7] << a*0.5, b*0.5, c*-0.5;
   
    for( int i = 0; i < 8; ++i ) {
      cv::Point p; Eigen::Vector3d p3;
      p3 = rot*boxVertices3D[i] + box_pos;

      // Transform to pixel coordinates      
      p.x = width/2 - (int)(p3(0)*f/p3(2));
      p.y = height/2 -(int)(p3(1)*f/p3(2));

     // Store
     boxVertices2D[i] = p;
    } // end for


  } else {
    printf("No detected boxes \n");
  }
}

/**
 * @function pickObject
 */
void pickObject( int state, void* userdata ) {
  *segmentedObject = clusters[selectedSegmentedCloud];
}

/**
 * @function sendBox
 */
void sendBox( int state, void* userdata ) {
  struct sns_msg_box msg;
  sns_msg_header_fill( &msg.header );
  msg.p[0] = box_pos(0); msg.p[1] = box_pos(1); msg.p[2] = box_pos(2);

  msg.q[0] = box_rot.x(); msg.q[1] = box_rot.y(); 
  msg.q[2] = box_rot.z(); msg.q[3] = box_rot.w();
  msg.s[0] = box_size(0); msg.s[1] = box_size(1);  
  msg.s[2] = box_size(2); msg.s[3] = box_size(3);

  ach_status r;
  r = ach_put( &box_param_chan, &msg, sizeof(msg) );
  if( r != ACH_OK ) {
    printf("\t * [BAD] Error sending box message \n");

  } else {
    printf("\t * [GOOD] Box message sent all right! \n");
  }


}

/**
 * @function sendObject
 */
void sendObject( int state, void* userdata ) {

  // Send through channel
  uint32_t n_points = clusters[selectedSegmentedCloud].points.size();
  struct sns_msg_segmented_cloud* msg = sns_msg_segmented_cloud_heap_alloc( n_points );
  
  sns_msg_segmented_cloud_init( msg, n_points );
  msg->n_clusters = 1;
  msg->selected = 0; // the only object we are sending


  int count = 0;
  
  for( int j = 0; j < clusters[selectedSegmentedCloud].size(); ++j ) {
    msg->u[count].x = clusters[selectedSegmentedCloud].points[j].x;
    msg->u[count].y = clusters[selectedSegmentedCloud].points[j].y;
    msg->u[count].z = clusters[selectedSegmentedCloud].points[j].z;
    msg->u[count].cluster = 0;
    count++;
  }
  

  ach_status r;
  printf("\t * Size of msg: %d \n", sns_msg_segmented_cloud_size(msg) );
  r = ach_put( &segmented_cloud_chan, msg,sns_msg_segmented_cloud_size(msg) );
  if( r != ACH_OK ) {
    printf("\t * [BAD] Error sending message. Probably OVERFLOW? Increase frame size? \n");

  } else {
    printf("\t * [GOOD] Message sent all right! \n");
  }

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
 * @function drawBox
 */
void drawBox() {

  cv::line( rgbImg, boxVertices2D[0], boxVertices2D[1], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[1], boxVertices2D[2], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[2], boxVertices2D[3], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[3], boxVertices2D[0], cv::Scalar(0,0,0), 2, 8 );  

  cv::line( rgbImg, boxVertices2D[4], boxVertices2D[5], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[5], boxVertices2D[6], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[6], boxVertices2D[7], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[7], boxVertices2D[4], cv::Scalar(0,0,0), 2, 8 );  

  cv::line( rgbImg, boxVertices2D[0], boxVertices2D[4], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[1], boxVertices2D[5], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[2], boxVertices2D[6], cv::Scalar(0,0,0), 2, 8 );  
  cv::line( rgbImg, boxVertices2D[3], boxVertices2D[7], cv::Scalar(0,0,0), 2, 8 );  

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
