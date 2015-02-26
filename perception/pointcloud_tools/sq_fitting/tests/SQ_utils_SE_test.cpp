/**
 * @function SQ_utils_SE_test.cpp
 */
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <SQ_utils.h>


int main( int argc, char* argv[] ) {

  double a1 = 1.5;
  double a2 = 1.5;
  double a3 = 1.5;
  double e1 = 1;
  double e2 = 1;
  int N = 400;
  int v;

  while( (v=getopt(argc, argv, "n:a:b:c:e:f:")) != -1 ) {
    switch(v) {

    case 'a' : {
      a1 = atof(optarg);
    } break;
    case 'b' : {
      a2 = atof(optarg);
    } break;
    case 'c' : {
      a3 = atof(optarg);
    } break;
    case 'n' : {
      N = atoi(optarg);
    } break;
    case 'e' : {
      e1 = atof(optarg);
    } break;
    case 'f' : {
      e2 = atof(optarg);
    } break;

    } // switch end
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
 
  std::cout << "a1: "<< a1 << " a2: "<< a2 << " a3: "<< a3 << " e1: "<< e1<<" e2: "<< e2 << std::endl;
  std::cout << " num points: "<< N << std::endl;
    cloud = sampleSQ_uniform( a1, a2, a3, e1, e2, N );
  
  std::cout << "Cloud size: "<< cloud->points.size() << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("SQ_utils_SE_test Viewer") );
  viewer->addCoordinateSystem(1.0, 0);
  
  // 2. Visualize super ellipse
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col(cloud, 0,255,0);
  viewer->addPointCloud( cloud, col, "Superellipse" );

  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
  }
  
  return 0;
}
