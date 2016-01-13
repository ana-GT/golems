/**
 * @file grab_pointcloud.cpp
 */
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

// Create function to store cloud every time a key is pressed
pcl::visualization::CloudViewer viewer("Grab pointcloud");
bool storeCloud = false;



void cloudCallback( const CloudConstPtr& _cloud ) {

  // Show
  viewer.showCloud( _cloud );
  // Check if we need to store
  if( storeCloud ) {
    printf("Storing cloud \n");
    pcl::io::savePCDFileASCII("grabbed_full_pcd.pcd", *_cloud );
    storeCloud = false;
  }
}

void keyboardCallback( const pcl::visualization::KeyboardEvent &_event,
		       void* data ) {
  
  if( _event.getKeySym() == "s" && _event.keyDown() ) {
    // Store cloud
    storeCloud = true;
  }

  
}

int main( int argc, char* argv[] ) {

  // Create grabber
  pcl::io::OpenNI2Grabber grabber("#1");
  boost::function<void (const CloudConstPtr& )> f = boost::bind( cloudCallback, _1 );
  grabber.registerCallback(f);
  viewer.registerKeyboardCallback( keyboardCallback, NULL );  
  
  
  // Loop
  grabber.start();
  while( !viewer.wasStopped() ) {
    boost::this_thread::sleep( boost::posix_time::seconds(1) );
  }

  grabber.stop();

  return 0;
}
