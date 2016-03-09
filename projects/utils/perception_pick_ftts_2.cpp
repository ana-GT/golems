/**
 * @file perception_pick_ftts_2.cpp
 * @brief Recognizes objects in bounding boxes and uses it to select what fitting to do
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
#include "perception/pointcloud_tools/tabletop_symmetry/mindGapper.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_t.h"
#include "perception/pointcloud_tools/sq_fitting/SQ_fitter_b.h"
#include "object_recognition/ObjectsDatabase.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include <mutex>

#include <ach.h>
#include <sns.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

// Global variables
std::string gWindowName = std::string("Perception Pick and Recognition");
cv::Mat gRgbImg, gPclMap;
double gF;

pcl::io::OpenNI2Grabber* gGrabber = NULL;
Fast_Tabletop_Segmentation<PointT> gTts;
std::mutex  mutex;

// Bounding box
std::vector<Eigen::VectorXd> gClustersBB;
int gSelectedCluster;
bool gShowSegmentation = false;
bool gMirror = true;
// Functions
static void onMouse( int event, int x, int y, int flags, void* userdata );
void grabber_callback( const CloudConstPtr& _cloud );
void drawBoundingBox();

void startComm( int state, void* userdata );
void send( int state, void* userData );

void fit_SQ( pcl::PointCloud<PointT>::Ptr _cluster, int _index,
	     SQ_parameters &_p,
	     int _type );
void create_table_mesh( pcl::PolygonMesh &_table_mesh,
			char _table_name_mesh[50] );

void mirrorState(int state, void* data);
void storeCloud( char* _name, int _index, pcl::PointCloud<PointT>::Ptr _cloud );

// Communication
bool gChanReady = false;
ach_channel_t gObj_param_chan; // Channel to send object param to planner
ach_channel_t gServer2Module_chan; // Channel to receive commands from server
ach_channel_t gModule2Server_chan; // Channel to send responses to server

ObjectsDatabase gOd;

void mirrorState(int _state, void* data) {
  gMirror = _state;
}

//////////////////////////
//  @function main
/////////////////////////
int main( int argc, char* argv[] ) {

  // Set configuration values for Tts (tabletop segmentor)
  gRgbImg = cv::Mat( 480, 640, CV_8UC3 );
  gTts.setMinClusterSize(300);  
  gTts.setZfilters( 0.35, 1.0 );
  
  // Set capture
  gGrabber = new pcl::io::OpenNI2Grabber("", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode, 
					 pcl::io::OpenNI2Grabber::OpenNI_Default_Mode );
  boost::function<void (const CloudConstPtr&) > f = boost::bind(&grabber_callback, _1 );
  gGrabber->registerCallback(f);

  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
  cv::setMouseCallback( gWindowName, onMouse, 0 );
  cv::createButton( "Mirror", mirrorState, NULL, cv::QT_CHECKBOX, gMirror );
  cv::createButton( "Start comm", startComm, NULL, cv::QT_PUSH_BUTTON, false );
  cv::createButton( "Send", send, NULL, cv::QT_PUSH_BUTTON, false );

  boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = gGrabber->getDevice();
  gF = device->getDepthFocalLength(); // Same as color length, by the way

// Object database
gOd.init_classifier();
gOd.load_dataset();

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

    // Recognize 
    int xmin, ymin, xmax, ymax; int index; std::string label;
    gTts.getClusterBB( i, xmin, ymin, xmax, ymax );
    gOd.classify( cv::Mat( gRgbImg, cv::Rect(xmin,ymin, xmax-xmin, ymax-ymin) ),
		  index, label );
    // Send this info for fitting
    fit_SQ( gTts.getCluster(i), i, p, gOd.getSQtype(index) );
    printf("* Label: %s \n", label.c_str() );
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
 * @function storeCloud
 */
void storeCloud( char* _name, int _index, pcl::PointCloud<PointT>::Ptr _cloud ) {
    char complete_name[100]; 
    sprintf(complete_name, "%s_%d.pcd", _name, _index );
    pcl::io::savePCDFile( complete_name, *_cloud );
}

/**
 * @function fit_SQ
 */
void fit_SQ( pcl::PointCloud<PointT>::Ptr _cluster, int _index,
	     SQ_parameters &_p, int _type ) {
  
  pcl::PointCloud<PointT>::Ptr filtered( new pcl::PointCloud<PointT>() );  
  pcl::PointCloud<PointT>::Ptr completed( new pcl::PointCloud<PointT>() );  

  // Filter
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (_cluster);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*filtered);

  Eigen::Isometry3d Tsymm; Eigen::Vector3d Bb;
  *completed = *filtered;

  // Mirror
  if( gMirror ) {
    mindGapper<PointT> mg;
    std::vector<double> tc(4); for( int i = 0; i < 4; ++i ) { tc[i] = gTts.mTableCoeffs[i]; }
    mg.setTablePlane( tc );
    mg.setFittingParams();
    mg.setDeviceParams();
    mg.setFocalDist(gF);

    mg.reset();
    // Debug: Store completed and incompleted clouds
    storeCloud( "raw", _index, completed );    
    mg.complete( completed );
    storeCloud( "mirrored", _index, completed );
    mg.getSymmetryApprox( Tsymm, Bb );
  }

  // Fit superquadric
  switch( _type ) {
  case REGULAR: {
    printf("Type: regular \n");
    SQ_fitter<PointT> fitter;
    fitter.setInputCloud( completed );
    if( gMirror ) { fitter.setInitialApprox( Tsymm, Bb ); }
    fitter.fit( SQ_FX_ICHIM, 0.03, 0.005, 5, 0.1 );
    fitter.getFinalParams( _p );
    break;
  }
  case TAMPERED: {
    printf("Type: tampered \n");
    SQ_fitter_t<PointT> fitter;
    fitter.setInputCloud( completed );
    if( gMirror ) { fitter.setInitialApprox( Tsymm, Bb ); }
    fitter.fit( SQ_FX_ICHIM, 0.03, 0.005, 5, 0.1 );
    fitter.getFinalParams( _p );
    break;
  }
  case BENT: {
    printf("Type: bent \n");
    SQ_fitter_b<PointT> fitter;
    fitter.setInputCloud( completed );
    if( gMirror ) { fitter.setInitialApprox( Tsymm, Bb ); }
    fitter.fit( SQ_FX_ICHIM, 0.03, 0.005, 5, 0.1 );
    fitter.getFinalParams( _p );
    break;
  }
  }


    printParamsInfo(_p);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sqp( new pcl::PointCloud<pcl::PointXYZ>() );
    char sqname[75]; sprintf( sqname, "sq_pointcloud_%d.pcd", _index ); 
    char oname[75]; sprintf( oname, "original_%d.pcd", _index );
    sqp = sampleSQ_uniform<pcl::PointXYZ>( _p ); 
    pcl::io::savePCDFile( sqname, *sqp );
    pcl::io::savePCDFile( oname, *completed );
    
  
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
