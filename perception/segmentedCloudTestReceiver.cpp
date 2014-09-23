
#include <sys/stat.h>
#include <time.h>
#include <stdint.h>
#include <ach.h>
#include <sns.h>
#include <sns/msg.h>

#include "msgs/perception_msgs.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


ach_channel_t segmented_cloud_chan;
std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clusters;


void update();
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ( const std::vector< pcl::PointCloud<pcl::PointXYZRGBA> > &clouds );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  srand(time(NULL));

  sns_init();
  sns_start();

  // Open the channel
  sns_chan_open( &segmented_cloud_chan, "segmented-cloud", NULL );

  while( !sns_cx.shutdown ) {  
    update();
    aa_mem_region_local_release();
  }
  
}

void update() {

  size_t frame_size;
  ach_status r;
  struct sns_msg_segmented_cloud* msg;
  

  r = sns_msg_local_get( &segmented_cloud_chan,
			 (void**) &msg,
			 &frame_size,
			 NULL, ACH_O_LAST );

  if( ACH_OK == r || ACH_MISSED_FRAME == r ) {
    printf("Received segmented information \n");
    printf("Number of segmented clouds: %d \n", msg->header.n );
    printf("Number of clusters: %d \n", msg->n_clusters );

    clusters.resize( msg->n_clusters );
    for( int i = 0; i < clusters.size(); ++i ) {
      clusters[i].points.resize(0);
    }

    std::vector<Eigen::Vector3i> colors( clusters.size() );
    for( int i = 0; i < clusters.size(); ++i ) {
      Eigen::Vector3i v;
      v << rand() % 255, rand() % 255, rand() % 255;
      colors.push_back(v);
    }

    for(int i = 0; i < msg->header.n; ++i ) {
      pcl::PointXYZRGBA p;
      p.x =  msg->u[i].x;
      p.y = msg->u[i].y;
      p.z = msg->u[i].z;
      p.r = colors[msg->u[i].cluster](0);
      p.g = colors[msg->u[i].cluster](1);
      p.b = colors[msg->u[i].cluster](2);
      p.a = 255;
      clusters[ msg->u[i].cluster ].push_back( p );     
    }

    // View 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(clusters);

    while (!viewer->wasStopped ()) {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    
  } //

}



boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ( const std::vector< pcl::PointCloud<pcl::PointXYZRGBA> > &clouds ) {
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  
  for( int i = 0; i < clouds.size(); ++i ) {
    char name[20];
    sprintf(name, "cloud_%d", i );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA>() );
    *cloud = clouds[i];
    viewer->addPointCloud<pcl::PointXYZRGBA> ( cloud, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name );
  }
    
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
