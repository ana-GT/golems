
#include <pcl/io/pcd_io.h>
#include <perception/pointcloud_tools/sq_fitting/SQ_utils.h>
#include <perception/pointcloud_tools/sq_fitting/SQ_fitter_b.h>


typedef pcl::PointXYZ PointT;

int main( int argc, char* argv[] ) {

  pcl::PointCloud<PointT>::Ptr bent( new pcl::PointCloud<PointT>() );
  pcl::PointCloud<PointT>::Ptr bent_small( new pcl::PointCloud<PointT>() );
  SQ_parameters par;
  par.dim[0] = 0.02; par.dim[1] = 0.02; par.dim[2] = 0.12;
  par.e[0] = 0.1; par.e[1] = 1.0;
  par.type = BENT;
  par.k = 10.5; // 0.12
  par.alpha = 0;
  par.rot[0] = 0.89; par.rot[1] = -1.5; par.rot[2] = 0.9;
  par.trans[0] = -0.05; par.trans[1] = 0.1; par.trans[2] = 0.57;

   bent = sampleSQ_uniform_b2<PointT>( par );
  pcl::io::savePCDFileASCII( "bent_test.pcd", *bent );

  downsampling<PointT>( bent, 0.01, bent_small );

  // Try to fit
  SQ_fitter_b<PointT> fitter;
  fitter.setInputCloud(bent_small );
  fitter.fit( SQ_FX_ICHIM, 0.03, 0.005, 5, 0.001 );
  SQ_parameters p;
  fitter.getFinalParams(p);
  printParamsInfo(p);
  // Create a random 
  pcl::PointCloud<PointT>::Ptr sqp( new pcl::PointCloud<PointT>() );
  sqp = fitter.getSampledOutput();
  pcl::io::savePCDFile("sq_testing.pcd", *sqp );
}
