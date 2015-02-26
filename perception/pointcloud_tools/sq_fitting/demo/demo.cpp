/**
 * @file demo.cpp
 * @brief Demo of super quadric fitting code
 * @author A. Huaman Quispe <ahuaman3@gatech.edu>
 */
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <tabletop_segmentation/tabletop_segmentation.h>
#include <tabletop_symmetry/mindGapper.h>

//-- Global variables
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gCloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA> );
pcl::PointCloud<pcl::PointXYZ>::Ptr gFallbackCloudPtr( new pcl::PointCloud<pcl::PointXYZ> );
boost::shared_ptr<pcl::visualization::CloudViewer> gViewer;

pcl::Grabber* gKinectGrabber;
unsigned int gFilesSaved = 0;
bool gProcessCloud = false;
bool gNoColour = false;

TabletopSegmentor<pcl::PointXYZRGBA> gTs;

//-- Functions declaration
void printUsage( char* argv0 );
void grabberCallback( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud );
void keyboardEventOccurred( const pcl::visualization::KeyboardEvent &_event,
			    void *_nothing );
boost::shared_ptr<pcl::visualization::CloudViewer> createViewer();

/**
 * @function main
 * @brief Main function what else?
 */
int main( int argc, char* argv[] ) {

  int c;
  while( (c=getopt(argc, argv,"h")) != -1 ) {
    switch(c) {
    case 'h': { printUsage(argv[0]); } break;	
    }
  }
  
  // Create OpenNI grabber, register key events
  gKinectGrabber = new pcl::OpenNIGrabber();
  if( gKinectGrabber == 0 ) { return 1; }
  boost::function< void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind( &grabberCallback, _1 );
  gKinectGrabber->registerCallback(f);
  
  // Create viewer and initialize
  gViewer = createViewer();
  gKinectGrabber->start();

  // Loop expecting capture signal (press ESC)
  while( !gViewer->wasStopped() ) {
    boost::this_thread::sleep( boost::posix_time::seconds(1) );
  }
  
  // If capturing Kinect data, stop the stream before exiting
  gKinectGrabber->stop();
  
} // end main



/**
 * @function printUsage
 */
void printUsage( char* argv0 ) {
  std::cout<< "**  Usage:" << std::endl;
  std::cout<< argv0 <<" [ARGS] "<< std::endl;
  std::cout<<" <none>: : Start capturing from a Kinect device, save by pressing SPACE"<<std::endl;
  std::cout<<" -h : Show this help"<<std::endl;;
}

/**
 * @function grabberCallback
 */
void grabberCallback( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud ) {
  
  if( !gViewer->wasStopped() ) {
    gViewer->showCloud( _cloud );
  }
  
  if( gProcessCloud ) {
    
    std::stringstream stream;
    stream << "inputCloud" << gFilesSaved << ".pcd";
    std::string filename = stream.str();
    
    if( pcl::io::savePCDFile( filename, *_cloud, true ) == 0 ) {
      gFilesSaved++;
      std::cout << "Saved " << filename << "." << std::endl;
    }
    else {
      PCL_ERROR( "[!]Problem saving %s. \n", filename.c_str() );
    }

    /** Process cloud */
    gTs.processCloud( _cloud );

    // Get camera parameters
    double f, cx, cy;
    int width, height;

    XnMapOutputMode m = ((pcl::OpenNIGrabber*)gKinectGrabber)->getDevice()->getDepthOutputMode();
    width = (int) m.nXRes;
    height = (int) m.nYRes;
    
    f = (double)((pcl::OpenNIGrabber*)gKinectGrabber)->getDevice()->getDepthFocalLength(0);
    cx = width >> 1;
    cy = height >> 1;

    
    /** Set mindGapper (to generate mirror pointclouds)*/

    mindGapper<pcl::PointXYZRGBA> mG;
    mG.setTablePlane( gTs.getTableCoeffs() );
    mG.setFittingParams( 6,5,0.01,M_PI/9.0 );
    mG.setDeviceParams( width, height, f, (double)cx, (double)cy );

    /** Generate mirror pointclouds */

    for( int i = 0; i < gTs.getNumClusters(); ++i ) {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster( new pcl::PointCloud<pcl::PointXYZRGBA>() );
      *cluster = gTs.getCluster(i);
      mG.complete( cluster );
      char name[50]; sprintf( name, "mirror_%d.pcd", i );
      pcl::io::savePCDFile( name, *cluster, true );
    }

    gProcessCloud = false;
  }

}

/**
 * @function keyboardEventOccurred
 */
void keyboardEventOccurred( const pcl::visualization::KeyboardEvent &_event,
			   void *_nothing ) {
  
  // Save cloud when pressing space
  if( _event.getKeySym() == "space" && _event.keyDown() ) {
    gProcessCloud = true;
  }
}

/**
 * @function createViewer
 */
boost::shared_ptr<pcl::visualization::CloudViewer> createViewer() {
  boost::shared_ptr<pcl::visualization::CloudViewer> v( new pcl::visualization::CloudViewer("3D Viewer") );
  v->registerKeyboardCallback( keyboardEventOccurred );

  return (v);
}
