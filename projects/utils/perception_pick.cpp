/**
 * @file perception_pick
 * @brief Captures a snapshot, segments and select the type of primitive you want to use to represent the objects
 * @brief It can be controlled by msgs from server
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
#include <communication/server_pick/include/fsm_msg.h>
#include <communication/server_pick/include/fsm_header.h>

#include <global/crichton_global.h>
#include <global/fsa_data.h>

#include <stdlib.h>
#include <time.h>

#include <pcl/surface/poisson.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/ply_io.h>


// SQ, BB
#include "refresher_utils/Refresher_utils.h"
#include "sq_fitting/SQ_utils.h"
#include "sq_fitting/SQ_fitter.h"
#include "tabletop_symmetry/mindGapper.h"


typedef pcl::PointXYZRGBA PointTa;

/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::string gWindowName = std::string("Perception Pick");
cv::Mat gRgbImg;
cv::Mat gPclMap;

ach_channel_t gSegmented_cloud_chan;
Eigen::Vector3d gCurrentPoint;
bool gIsSegmentedFlag = false;
double gF;

int gSelectedSegmentedCloud = -1;
std::vector<pcl::PointCloud<PointTa> > gClusters;
std::vector<cv::Vec3b> gColors;
std::vector< std::vector<Eigen::Vector2d> > gPixelClusters;
std::vector<Eigen::Vector2d> gClusterCentroids;
std::vector<double> gTableCoeffs;
pcl::PointCloud<PointTa> gTablePoints;
bool gChanReady = false;

// CHANNELS
ach_channel_t gObj_param_chan; // Channel to send object param to planner
ach_channel_t gServer2Module_chan; // Channel to receive commands from server
ach_channel_t gModule2Server_chan; // Channel to send responses to server


/***********************/
/** FUNCTIONS          */
/***********************/
static void onMouse( int event, int x, int y, int flags, void* userdata );
void process( int state, void* userdata );

void startComm( int state, void* userdata );
void setSQ( int state, void* userData );
void send( int state, void* userData );
void fit_SQ( pcl::PointCloud<PointTa> _cluster, int i,
	     double dim[3], double trans[3], double rot[3], double e[2] );

void create_mesh( pcl::PolygonMesh &_mesh,
		  double _dim[3],
		  double _e[2], int _N,
		  char* _mesh_name );

void fix_mesh_faces( const pcl::PointCloud<pcl::PointXYZ> &_points,
		     std::vector<pcl::Vertices> &_polygons );

void create_table_mesh( pcl::PolygonMesh &_table_mesh,
			char _table_name_mesh[50] );

void drawSegmented();
void getPixelClusters();
void pollChan(); 

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Initialization
  srand( time(NULL) );
  
  // Open device
  cv::VideoCapture capture( cv::CAP_OPENNI2 );
  
  if( !capture.isOpened() ) {
    printf( "\t * Could not open the capture object \n" );
    return -1;
  }

  printf("\t * Opened device \n");
  
  capture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 ); // off
  capture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 ); // on
  
  printf("\t * Common Image dimensions: (%f,%f) \n",
	 capture.get( cv::CAP_PROP_FRAME_WIDTH ),
	 capture.get( cv::CAP_PROP_FRAME_HEIGHT ) );
  

  // Set control panel
  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
  
  int value;
  cv::createTrackbar("track1", gWindowName.c_str(), &value, 255, NULL, NULL );

  cv::createButton( "Start comm", startComm, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Process", process, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );
  cv::createButton( "Send", send, 
		    NULL, cv::QT_PUSH_BUTTON,
		    false );

  
  // Set mouse callback 
  cv::setMouseCallback( gWindowName, onMouse, 0 );

  // Loop
  gF = (float)capture.get( cv::CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH );
 
  for(;;) {
    
    if( !capture.grab() ) {
      printf( "\t * [ERROR] Could not grab a frame \n" );
      return -1;
    }
    
    capture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    if( gIsSegmentedFlag ) {
      drawSegmented();
    }
    cv::imshow( gWindowName, gRgbImg );
    
    capture.retrieve( gPclMap, cv::CAP_OPENNI_POINT_CLOUD_MAP );
    
    if( gChanReady ){ pollChan(); }
    

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t * [PRESSED ESC] Finishing the program \n");
      break;
    }

  } // end for
  
  return 0;
}

/**
 * @function pollChan
 */
void pollChan() {

  // Check master to server

  // Server2See
  ach_status_t r;
  size_t frame_size;
  alpha_msg server_msg;
  alpha_msg module_msg;
  
  r = ach_get( &gServer2Module_chan, &server_msg,
	       sizeof(server_msg), &frame_size, NULL, ACH_O_LAST );

  if( r == ACH_OK && !sns_msg_is_expired( &server_msg.header, NULL ) ) {
    printf("Got message of type %d \n", server_msg.type );
    switch( server_msg.type ) {

    case PICKUP_MSG_CMD_PERCEPTION_GRAB_IMG : {

      process(0, NULL );
      if( gIsSegmentedFlag ) {
	drawSegmented();
      }
      // Store image with a random number attached
      int number = rand() % 1000;
      char fullName[255];
      char imgName[20];
      sprintf( imgName, "rgbImg_%d.png", number );
      sprintf( fullName, "%s/rgbImg_%d.png", gPicturesPath.c_str(),  number);
      
      cv::imwrite( fullName, gRgbImg );
      usleep( 0.1*1e6 );
      
      // Send message replying
      module_msg.type = PICKUP_MSG_STATUS_PERCEPTION_IMG_YES;
      sns_msg_set_time( &module_msg.header, NULL, 0.2*1e9 );
      sprintf( module_msg.line, "%d %s", module_msg.type, imgName );
      
      r = ach_put( &gModule2Server_chan, &module_msg, sizeof(module_msg) );
      if( r == ACH_OK ) {
	printf("Msg back to server sent all right \n");
      } else {
	printf("[ERROR] Msg back to server NOT sent well");
      }
    } break;

      
    case PICKUP_MSG_CMD_PERCEPTION_SEND :  {
      
      // Get x and y to select an object
      int t, px, py;
      std::stringstream ss(server_msg.line);
      ss >> t >> px >> py;
      printf("X: %d Y: %d \n", px, py);
      onMouse( cv::EVENT_LBUTTONDOWN,
	       px, py, 0, NULL );
      // Send it
      send( 0, NULL );
      usleep(0.5*1e6);
      
      // Send message replying
      module_msg.type = PICKUP_MSG_STATUS_PERCEPTION_SENT_YES;
      sns_msg_set_time( &module_msg.header, NULL, 0.2*1e9 );
      r = ach_put( &gModule2Server_chan, &module_msg, sizeof(module_msg) );
      
    } break;

      
    } // end switch
      

  } // end if
    
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
 * @function onMouse
 * @brief Stores the current position of clicked point 
 */
static void onMouse( int event, int x, int y, int, void* ) {
  
  if( event != cv::EVENT_LBUTTONDOWN ) {
    return;
  }

  // Get (X,Y,Z) from Kinect
  cv::Point3f p; Eigen::Vector3d currentPoint;
  p = gPclMap.at<cv::Point3f>(y,x);
  currentPoint << (double)-1*p.x, (double)p.y, (double)p.z;

  std::cout << "\t * [INFO] Current point: "<< currentPoint.transpose() << std::endl;

  // Check what segmented object is selected
  if( gClusterCentroids.size() > 0 ) {
    Eigen::Vector2d p; p << (double)x, (double) y;
    double dist;
    double minDist = 1000; int minInd = -1;
    for( int i = 0; i < gClusterCentroids.size(); ++i ) {
      dist = ( p - gClusterCentroids[i] ).norm();
      if( dist < minDist ) {
	minDist = dist; minInd = i; 
      }
    }
    
    gSelectedSegmentedCloud = minInd;

    printf( "\t [INFO] Segmented cloud to send: %d \n", gSelectedSegmentedCloud );
  } // end if
}



/**
 * @function send
 */
void send( int state, void* userData ) {
  
  printf("Clusters size: %d \n", gClusters.size() );
  if( gSelectedSegmentedCloud < 0 || gSelectedSegmentedCloud >= gClusters.size() ) {
    printf("--> ERROR: Did not select a cloud? \n");
    return;
  }

  // 0. Set message alloc
  sns_msg_tabletop_sq* msg = sns_msg_tabletop_sq_alloc( gClusters.size() );
  sns_msg_header_fill( &msg->header );
  msg->header.n = (uint32_t)gClusters.size();
  msg->n = (uint32_t) gClusters.size();
  msg->i = (uint32_t) gSelectedSegmentedCloud;

  // Variables 
  pcl::PolygonMesh mesh;
  pcl::PointCloud<PointTa>::Ptr mirrored_cloud;
  size_t i;
  double dim[3]; double trans[3]; double rot[3]; double e[2];  
  
  
  // 1. Save SQs
  for( i = 0; i < gClusters.size(); ++i ) {  
    fit_SQ( gClusters[i], i, dim, trans, rot, e );
    for(int j = 0; j < 3; ++j ) { msg->u[i].dim[j] = dim[j]; }
    for( int j = 0; j < 3; ++j ) { msg->u[i].trans[j] = trans[j]; }
    for( int j = 0; j < 3; ++j ) { msg->u[i].rot[j] = rot[j]; }
    for( int j = 0; j < 2; ++j ) { msg->u[i].e[j] = e[j]; }

    char mesh_name[50]; sprintf( mesh_name, "%s/mesh_%d.ply", gPicturesPath.c_str(), i );
    create_mesh( mesh, dim, e, 25, mesh_name );
    msg->u[i].mesh_generated = true;
    strcpy( msg->u[i].mesh_filename, mesh_name );        
  }
  
  // 2. Get the table
  char tableName[50]; sprintf( tableName, "%s/tableMesh.ply", gPicturesPath.c_str() );
  create_table_mesh( mesh, tableName );
  
  pcl::io::savePLYFile( tableName, mesh ); 
  
  for( i = 0; i < 4; ++i ) { msg->table_coeffs[i] = gTableCoeffs[i]; }
  sprintf( msg->table_meshfile, tableName );

  ach_status_t r = ach_put( &gObj_param_chan, msg, sns_msg_tabletop_sq_size(msg) );
  if( r != ACH_OK ) {
    printf("\t [BAD] Error sending message \n");
  } else {
    printf("\t [GOOD] Message sent all right \n ");
  }
  
  msg = 0;
}

/**
 * @function create_table_mesh
 */
void create_table_mesh( pcl::PolygonMesh &_table_mesh,
		   char _table_name_mesh[50] ) {

  pcl::Poisson<pcl::PointNormal> poisson;

  
  pcl::PointCloud<pcl::PointNormal>::Ptr pnt( new pcl::PointCloud<pcl::PointNormal>() );
  for( int i = 0; i < gTablePoints.points.size(); ++i ) {
    pcl::PointNormal p;
    p.x = gTablePoints.points[i].x; p.y = gTablePoints.points[i].y; p.z = gTablePoints.points[i].z;
    p.normal_x = gTableCoeffs[0]; p.normal_y = gTableCoeffs[1]; p.normal_z = gTableCoeffs[2];	
    pnt->points.push_back(p);
  }
  pnt->width = 1; pnt->height = pnt->points.size();
  
  poisson.setDepth(5);
  poisson.setInputCloud(pnt);
  poisson.reconstruct(_table_mesh);
  
  pcl::io::savePLYFile( _table_name_mesh, _table_mesh ); 
}

/**
 * @function create_mesh
 */
void create_mesh( pcl::PolygonMesh &_mesh,
		  double _dim[3],
		  double _e[2], int _N,
		  char* _mesh_name ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_down( new pcl::PointCloud<pcl::PointXYZ>() );

  p = sampleSQ_uniform<pcl::PointXYZ>( _dim[0], _dim[1], _dim[2], _e[0], _e[1], _N );
  downsampling<pcl::PointXYZ>( p, 0.0025, p_down );

  // Convex Hull
  pcl::ConvexHull<pcl::PointXYZ> chull;
  pcl::PointCloud<pcl::PointXYZ> points;
  std::vector<pcl::Vertices> polygons;
  
  chull.setInputCloud( p_down );
  chull.reconstruct( points, polygons );

  fix_mesh_faces( points, polygons );

  // Save mesh
  pcl::PCLPointCloud2 points2;
  pcl::toPCLPointCloud2<pcl::PointXYZ>( points, points2 );  
  _mesh.cloud = points2;
  _mesh.polygons = polygons;
  pcl::io::savePLYFile( _mesh_name, _mesh);
  
}

/**
 * @function fix_mesh_faces
 */
void fix_mesh_faces( const pcl::PointCloud<pcl::PointXYZ> &_points,
		     std::vector<pcl::Vertices> &_polygons ) {

  pcl::PointXYZ p1, p2, p3;
  Eigen::Vector3d b;
  Eigen::Vector3d p21, p31;
  Eigen::Vector3d N;
  int v2, v3;
  for( std::vector<pcl::Vertices>::iterator it = _polygons.begin();
       it != _polygons.end(); ++it ) {
    
    if( (*it).vertices.size() != 3 ) {
      continue;
    }

    p1 = _points[ (*it).vertices[0] ];
    p2 = _points[ (*it).vertices[1] ];
    p3 = _points[ (*it).vertices[2] ];

    // Find baricenter
    b << ( p1.x + p2.x + p3.x )/ 3.0, ( p1.y + p2.y + p3.y )/ 3.0, ( p1.z + p2.z + p3.z )/ 3.0;
    
    // Find normal
    p21 << p2.x - p1.x, p2.y - p1.y, p2.z - p1.z;
    p31 << p3.x - p1.x, p3.y - p1.y, p3.z - p1.z;
    N = p21.cross( p31 );

    // If
    if( b.dot(N) < 0 ) {
      v2 = (*it).vertices[1];
      v3 = (*it).vertices[2];
      (*it).vertices[1] = v3;
      (*it).vertices[2] = v2;
    }
    
  } // end for

}



/**
 * @function fit_SQ
 */
void fit_SQ( pcl::PointCloud<PointTa> _cluster, int _index,
	     double _dim[3], double _trans[3],
	     double _rot[3], double _e[2] ) {

  // Generate a mirror version of the pointcloud
  mindGapper<PointTa> mg;
  mg.setTablePlane( gTableCoeffs );
  mg.setFittingParams();
  mg.setDeviceParams();
  mg.setFocalDist(gF);
  
  // Fit pointcloud to superquadric
  SQ_fitter< PointTa> fitter;
  pcl::PointCloud<PointTa>::Ptr completed( new pcl::PointCloud<PointTa>() );
  Eigen::Isometry3d Tsymm; Eigen::Vector3d Bb;
  
  *completed = _cluster;
  mg.reset();
  mg.complete( completed );
  mg.getSymmetryApprox( Tsymm, Bb );
  
  fitter.setInputCloud( completed );
  fitter.setInitialApprox( Tsymm, Bb );
  if( fitter.fit( SQ_FX_ICHIM, 0.03, 0.005, 5, 0.1 ) ) {
    
    SQ_parameters p;
    fitter.getFinalParams( p );
    for( int i = 0; i < 3; ++i ) { _dim[i] = p.dim[i]; }
    for( int i = 0; i < 3; ++i ) { _trans[i] = p.trans[i]; }
    for( int i = 0; i < 3; ++i ) { _rot[i] = p.rot[i]; }
    for( int i = 0; i < 2; ++i ) { _e[i] = p.e[i]; }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sqp( new pcl::PointCloud<pcl::PointXYZ>() );
    char sqname[75]; sprintf( sqname, "sq_pointcloud_%d.pcd", _index );  
    sqp = sampleSQ_uniform<pcl::PointXYZ>( p ); 
    pcl::io::savePCDFile( sqname, *sqp );

  }
  
}


/**
 * @function process
 */
void process( int state, 
	      void* userdata ) {

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
  pcl::io::savePCDFile( "debugPcl.pcd", *cloud, true );
  
  
  // Segment
  TabletopSegmentor<PointTa> tts;
  tts.set_filter_minMax( -0.75, 0.75, -0.75, 0.75, 0.25, 0.9 );
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
    
    cv::Vec3b def;
    def(0) = rand() % 255; def(1) = rand() % 255; def(2) = rand() % 255;  
    gColors[i] = def;
  }
  getPixelClusters();
  
}


/**
 * @function drawSegmented
 */
void drawSegmented() {
  for( int i = 0; i < gPixelClusters.size(); ++i ) {
    for( int j = 0; j < gPixelClusters[i].size(); ++j ) {
      gRgbImg.at<cv::Vec3b>( gPixelClusters[i][j](1), gPixelClusters[i][j](0) ) = gColors[i];
    }
  }
  
}

/**
 * @function getPixelClusters
 */
void getPixelClusters() {

  gPixelClusters.resize( gClusters.size() );
  gClusterCentroids.resize( gClusters.size() );
  
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

    for( pcl::PointCloud<PointTa>::iterator it = gClusters[i].begin();
	 it != gClusters[i].end(); ++it ) {

      X = (*it).x; Y = (*it).y; Z = (*it).z;     
      u = width/2 - (int)(X*gF/Z);
      v = height/2 -(int)(Y*gF/Z);
      
      gPixelClusters[i].push_back( Eigen::Vector2d(u,v) );

      sum_u += u;
      sum_v += v;
    }
    
    Eigen::Vector2d ct;
    ct << (double)(sum_u)/gClusters[i].points.size(), (double)(sum_v)/gClusters[i].points.size();
    gClusterCentroids[i] = ct;

  }

  
}
