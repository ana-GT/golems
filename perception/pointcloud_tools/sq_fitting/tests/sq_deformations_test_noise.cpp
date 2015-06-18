/**
 * @function SQ_utils_SE_test.cpp
 */
#include <pcl/io/pcd_io.h>
#include <SQ_utils.h>
#include <SQ_deformations.h>
#include <analytic_equations.h>
#include <sq_fitting/SQ_fitter.h>

int main( int argc, char* argv[] ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr tampered( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::io::loadPCDFile<pcl::PointXYZ> ( argv[1], *tampered ); 

  printf("Size input: %d \n", tampered->points.size() );

  // Fit perfect input
  SQ_fitter<pcl::PointXYZ> fitter;
  fitter.setInputCloud( tampered );
  if( fitter.fit_tampering( 0.03, 0.005, 5, 0.1 ) ) {
    printf("YAHOO! Fitted superquadric \n");
    fitter.printResults();
    SQ_parameters output;
    fitter.getFinalParams( output );
    printf("Tampered: %f \n", output.tamp );
    // Save recovered
    pcl::PointCloud<pcl::PointXYZ>::Ptr recovered;
   SQ_deformations sqd;
    recovered = sqd.linear_tampering( output.dim[0], output.dim[1],
				      output.dim[2], output.e[0],
				      output.e[1], output.tamp );
    pcl::io::savePCDFileASCII ( "recovered.pcd", *recovered );



  } else {
    printf("CRAP, did not fit! \n");

  }

  return 0;
}
