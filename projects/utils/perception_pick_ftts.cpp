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

#include <global/crichton_global.h>
#include <global/fsa_data.h>

#include "perception/ftts/fast_tabletop_segmentation.h"
#include "perception/msgs/perception_msgs.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_t.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_b.h"


#include <mutex>

#include <ach.h>
#include <sns.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

// Global variables
std::string gWindowName = std::string("Perception Pick 4");
cv::Mat gRgbImg, gPclMap;
double gF;

pcl::io::OpenNI2Grabber* gGrabber = NULL;
Fast_Tabletop_Segmentation<PointT> gTts;
std::mutex  mutex;

// Bounding box
std::vector<Eigen::VectorXd> gClustersBB;
int gSelectedCluster;
bool gShowSegmentation = false;

// Functions
static void onMouse( int event, int x, int y, int flags, void* userdata );
void grabber_callback( const CloudConstPtr& _cloud );
void drawBoundingBox();

void startComm( int state, void* userdata );
void send( int state, void* userData );

void fit_SQ( pcl::PointCloud<PointT> _cluster, int _index,
	     SQ_parameters &_p );
void create_table_mesh( pcl::PolygonMesh &_table_mesh,
			char _table_name_mesh[50] );

// Communication
bool gChanReady = false;
ach_channel_t gObj_param_chan; // Channel to send object param to planner
ach_channel_t gServer2Module_chan; // Channel to receive commands from server
ach_channel_t gModule2Server_chan; // Channel to send responses to server


//////////////////////////
//  @function main
/////////////////////////
int main( int argc, char* argv[] ) {

  // Set configuration values for Tts (tabletop segmentor)
  gRgbImg = cv::Mat( 480, 640, CV_8UC3 );
  gTts.setMinClusterSize(300);  
  //gTts.setMinMaxFilter( -1.0, 1.0, -1.0, 1.0, 0.35, 1.0 ); // PrimeSense
  gTts.setMinMaxFilter( -0.35, 0.35, -0.70, 0.70, 1.5, 2.4 ); // Kinect

  // Set capture
  printf("Start grabber \n");
  gGrabber = new pcl::io::OpenNI2Grabber(""); //"", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode, 
  //pcl::io::OpenNI2Grabber::OpenNI_Default_Mode );
  printf("Grabber was initialized \n");
  boost::function<void (const CloudConstPtr&) > f = boost::bind(&grabber_callback, _1 );
  gGrabber->registerCallback(f);

  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
  cv::setMouseCallback( gWindowName, onMouse, 0 );
  cv::createButton( "Start comm", startComm, NULL, cv::QT_PUSH_BUTTON, false );
cv::createButton( "Send", send, NULL, cv::QT_PUSH_BUTTON, false );

boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = gGrabber->getDevice();
gF = device->getDepthFocalLength(); // Same as color length, by the way

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
 * @function startComm
 */
void startComm( int state, void* userdata ) {

  sns_init();
  sns_start();
  ach_status_t r;
  
  r = ach_open( &gObj_param_chan, gObj_param_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { printf("[ERROR] COULD NOT OPEN OBJ PARAM CHAN"); return; }
  r = ach_open( &gServer2Module_chan, gServer2Module_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { printf("[ERROR] COULD NOT OPEN SERVER-2-SEE CHAN"); return; }
  r = ach_open( &gModule2Server_chan, gModule2Server_chan_name.c_str(), NULL );
  if( r != ACH_OK ) { printf("[ERROR] COULD NOT SEE-2-SERVER CHAN"); return; }
    
  printf("\t [OK] Communication stablished and ready to go \n");
  gChanReady = true;

}


/**
 * @function send
 */
void send( int state, void* userData ) {

  mutex.lock(); // Don't process anything while we are sending

  printf("Clusters size: %d \n", gTts.getNumClusters() );
  if( gSelectedCluster < 0 || gSelectedCluster >= gTts.getNumClusters() ) {
    printf("--> ERROR: Did not select a cloud? \n");
    return;
  }
  
  // 0. Set message alloc
  sns_msg_tabletop_sq* msg = sns_msg_tabletop_sq_alloc( gTts.getNumClusters() );
  sns_msg_header_fill( &msg->header );
  msg->header.n = (uint32_t)gTts.getNumClusters();
  msg->n = (uint32_t) gTts.getNumClusters();
  msg->i = (uint32_t) gSelectedCluster;

  // Variables 
  pcl::PolygonMesh mesh;
  pcl::PointCloud<PointT>::Ptr mirrored_cloud;
  size_t i;
  SQ_parameters p;
  
  
  // 1. Save SQs
  for( i = 0; i < gTts.getNumClusters(); ++i ) {  
    fit_SQ( *gTts.getCluster(i), i, p );
    
    copy_SQparam_msg( p, msg->u[i] );    

    char mesh_name[50]; sprintf( mesh_name, "%s/mesh_%ld.ply", gPicturesPath.c_str(), i );
    SQ_utils::create_SQ_mesh( mesh, p, 25, mesh_name );
    msg->u[i].mesh_generated = true;
    strcpy( msg->u[i].mesh_filename, mesh_name );        
  }
  
  // 2. Get the table
  char tableName[50]; sprintf( tableName, "%s/tableMesh.ply", gPicturesPath.c_str() );
  create_table_mesh( mesh, tableName );

  pcl::io::savePLYFile( tableName, mesh ); 
  printf("Saved table ply file \n");
  for( i = 0; i < 4; ++i ) { msg->table_coeffs[i] = gTts.mTableCoeffs[i]; }
  sprintf( msg->table_meshfile, "%s", tableName );

  ach_status_t r = ach_put( &gObj_param_chan, msg, sns_msg_tabletop_sq_size(msg) );
  if( r != ACH_OK ) {
    printf("\t [BAD] Error sending message \n");
  } else {
    printf("\t [GOOD] Message sent all right \n ");
  }
  
  msg = 0;

  mutex.unlock(); // Let the processing go on with its life
}


/**
 * @function create_table_mesh
 */
void create_table_mesh( pcl::PolygonMesh &_table_mesh,
		   char _table_name_mesh[50] ) {
  
  
  pcl::PointCloud<pcl::PointNormal>::Ptr pnt( new pcl::PointCloud<pcl::PointNormal>() );
  CloudPtr table = gTts.getTable();
  for( int i = 0; i < table->points.size(); ++i ) {
    pcl::PointNormal p;
    p.x = table->points[i].x; p.y = table->points[i].y; p.z = table->points[i].z;
    p.normal_x = gTts.mTableCoeffs[0]; p.normal_y = gTts.mTableCoeffs[1]; p.normal_z = gTts.mTableCoeffs[2];	
    pnt->points.push_back(p);
  }
  pnt->height = 1; pnt->width = pnt->points.size();


  // Convex Hull
  pcl::ConvexHull<pcl::PointNormal> chull;
  pcl::PointCloud<pcl::PointNormal> points;
  std::vector<pcl::Vertices> polygons;
  
  chull.setInputCloud( pnt );
  chull.reconstruct( _table_mesh );

  pcl::io::savePLYFile( _table_name_mesh, _table_mesh ); 
}



/**
 * @function fit_SQ
 */
void fit_SQ( pcl::PointCloud<PointT> _cluster, int _index,
	     SQ_parameters &_p ) {
  /*
  // Generate a mirror version of the pointcloud
  mindGapper<PointTa> mg;
  mg.setTablePlane( gTableCoeffs );
  mg.setFittingParams();
  mg.setDeviceParams();
  mg.setFocalDist(gF);
  printf("Mirror version \n");
  */
  // Fit pointcloud to superquadric
  SQ_fitter< PointT> fitter;
  pcl::PointCloud<PointT>::Ptr completed( new pcl::PointCloud<PointT>() );
  Eigen::Isometry3d Tsymm; Eigen::Vector3d Bb;
  
  *completed = _cluster;
  printf("Size of cluster: %lu \n", completed->points.size() );
/*  mg.reset();
    mg.complete( completed );

  mg.getSymmetryApprox( Tsymm, Bb );
  printf("Fitting \n");*/
  fitter.setInputCloud( completed );
  //fitter.setInitialApprox( Tsymm, Bb );  
  //if( fitter.fit( SQ_FX_ICHIM, 0.03, 0.005, 1, 0.001 ) ) {
  fitter.fit( SQ_FX_ICHIM, 0.03, 0.005, 5, 0.001 );
    fitter.getFinalParams( _p );
    printParamsInfo(_p);
    printf("SHOULD BE SAVING \n");
    pcl::PointCloud<PointT>::Ptr sqp( new pcl::PointCloud<PointT>() );
    char sqname[75]; sprintf( sqname, "sq_pointcloud_%d.pcd", _index ); 
    char oname[75]; sprintf( oname, "original_%d.pcd", _index );
    sqp = fitter.getSampledOutput(); 
    pcl::io::savePCDFile( sqname, *sqp );
    pcl::io::savePCDFile( oname, *completed );
    
    // }
  
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
  p = gPclMap.at<cv::Point3f>(y,x);
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
    printf("\t * Selected cluster: %d \n", gSelectedCluster );
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
  gPclMap = gTts.getXyzImg();
  // Draw bounding box
  drawBoundingBox();
  mutex.unlock();
}

/**
 * @function drawingBoundingBox
 * @brief BB is a bit larger than it should ( pixels)
 */
void drawBoundingBox() {

  int xmin, ymin, xmax, ymax;

  cv::Vec3b colors; colors(0) = 255; colors(1) = 0; colors(2) = 0;
  gClustersBB.resize(0);

  for( int i = 0; i < gTts.getNumClusters(); ++i ) {
    gTts.getClusterBB( i, xmin, ymin, xmax, ymax );
    // Draw a bit bigger
    int dx = 2; int dy = 2;
    if( xmin - dx > 0 ) { xmin = xmin - dx; }
    if( xmax + dx < 640 - 1 ) { xmax = xmax + dx; }
    if( ymin -dy > 0 ) { ymin = ymin - dy; }
    if( ymax + dy < 480 - 1 ) { ymax = ymax + dy; }
    cv::rectangle( gRgbImg, cv::Point( xmin, ymin), cv::Point(xmax, ymax), colors, 2 );    
    gClustersBB.push_back( Eigen::Vector4d(xmin+1,ymin+1,xmax-1,ymax-1) );
  }

}
