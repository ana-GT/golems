/**
 * @file es_tests_4b.cpp
 * @brief Test mirror results from groups in visualization
 */
#include "evaluated_eqs.h"
#include <SQ_utils.h>
#include <SQ_fitter_evaluated.h>
#include <tabletop_symmetry/mindGapper.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/shapes.h>


const int gnF = 5;
char* fx_names[gnF] = { "Radial", "Solina", "Ichim", "Chevalier", "F5"};
int fx_sq[gnF] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5};

std::string gGroupName;
double gF;
std::vector<double> gTableCoeffs(4);
int gT;
double gD = 0.01;
typedef pcl::PointXYZRGBA PointT;
std::ofstream gOutput;

pcl::PointCloud<PointT>::Ptr downsampling( const pcl::PointCloud<PointT>::Ptr &_input,
						  const double &_voxelSize );

void saveParams( std::ofstream &_output,
		 const SQ_parameters &_par,
		 double _t,
		 double _eg, double _er, 
                 double _e_e1, double _e_e2,
                 double _e_v );

/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  int v;
  while( (v=getopt(argc, argv, "n:h")) != -1 ) {
    switch(v) {

    case 'n' : {
      gGroupName = std::string(optarg);
    } break;
    case 'h' : {
      printf("Usage: %s -n GROUP_NAME \n", argv[0]);
      printf("Run this from inside data_july_18 \n");
      return 1;
    } break;

    } // switch end
  }

  // Start visualizer
/*
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viz"));
  viewer->initCameraParameters ();
  viewer->addCoordinateSystem(0.2);

  // Load table
  pcl::PointCloud<PointT>::Ptr table( new pcl::PointCloud<PointT>() );
  pcl::io::loadPCDFile<PointT> ("table_points.pcd", *table);
  viewer->addPointCloud( table, "table");
*/
  
  
  // Open the file with data regarding each pointcloud
  char dataname[100];
  sprintf( dataname, "%s/%s_data.txt", gGroupName.c_str(), gGroupName.c_str() );
  std::ifstream input( dataname, std::ifstream::in );

  // Read
  // First the focal distance
  input >> gF;
  // Second, the plane equation
  input >> gTableCoeffs[0] >>  gTableCoeffs[1] >>  gTableCoeffs[2] >>  gTableCoeffs[3];

  // Third, the number of instances inside
  input >> gT;

  // Set the mindGapper while we are at it
  mindGapper<PointT> mg;
  mg.setTablePlane( gTableCoeffs );
  mg.setFittingParams();
  mg.setDeviceParams();
  mg.setFocalDist(gF);
  Eigen::Isometry3d Tsymm; Eigen::Vector3d Bb;

  // Parameters for the fitting
  SQ_fitter_evaluated<PointT> sfe;
  SQ_parameters par, par_h;
  clock_t ts, tf; double dt;  

  // Open file output
  char output_name[50];
  sprintf( output_name, "%s_output.txt", gGroupName.c_str() );
  gOutput.open( output_name, std::ofstream::out );


  // Fourth, read the instances
  for( int i = 0; i < gT; ++i ) {
    std::string filename;
    // Read the name
    input >> filename;
    pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT>() );
    pcl::io::loadPCDFile<PointT> ( filename.c_str(), *cloud ); 
    pcl::PointCloud<PointT>::Ptr down( new pcl::PointCloud<PointT>() );      
    down = downsampling( cloud, gD );

    mg.reset();
    mg.complete( cloud, false );
    mg.getSymmetryApprox( Tsymm, Bb );
   

  pcl::PointCloud<pcl::PointXYZ>::Ptr approx( new pcl::PointCloud<pcl::PointXYZ>() );
  int n; n = cloud->points.size();
  double err, erg, erd;
  int ftype;

  for( int j = 0; j < gnF; ++j ) {
  
  ftype= fx_sq[j];
  // Regular
    sfe.setInitialApprox( Tsymm, Bb );
    sfe.setInputCloud( down );

  ts = clock();
  sfe.fit( ftype, 0.03, 0.005, 1, 0.01 );
  tf = clock();
  dt = (tf-ts) / (double) CLOCKS_PER_SEC;

  sfe.getFinalParams( par );
  error_metric<PointT>( par, cloud, erg, err, erd );
  saveParams( gOutput, par, dt, erg, err, 0, 0, 0 );   

  approx = sampleSQ_uniform<pcl::PointXYZ>( par );
  char rn[50]; sprintf( rn, "rn/%s_%d_%d.pcd", gGroupName.c_str(), ftype, i );
  pcl::io::savePCDFileASCII( rn, *approx );

  // Hierarchical
  sfe.setInitialApprox( Tsymm, Bb );
  sfe.setInputCloud( down );

  ts = clock();
  sfe.fit( ftype, 0.03, 0.005, 5, 0.01 );
  tf = clock();
  dt = (tf-ts) / (double) CLOCKS_PER_SEC;

  sfe.getFinalParams( par_h );
  approx = sampleSQ_uniform<pcl::PointXYZ>( par_h );
  char hn[50]; sprintf( hn, "hn/%s_%d_%d.pcd", gGroupName.c_str(), ftype, i );
  pcl::io::savePCDFileASCII( hn, *approx );

  error_metric<PointT>( par_h, cloud, erg, err, erd );
  saveParams( gOutput, par_h, dt, erg, err, 0, 0, 0 );   

  }

    // View pointcloud, point and cube
/*
  approx = sampleSQ_uniform( par );
  char an[50];
  sprintf( an, "approx_%d.pcd", i );
  viewer->addPointCloud( approx, an );
*/

/*
    viewer->addPointCloud( cloud, filename.c_str() );
    pcl::PointXYZ cp;
    cp.x = Tsymm.translation()(0);cp.y = Tsymm.translation()(1);cp.z = Tsymm.translation()(2);
    char sn[30]; char cn[30];
    sprintf( sn, "sphere_%d", i );
    sprintf(cn, "cube_%d", i );
    viewer->addSphere( cp, 0.005, 255,0,255, sn );
    Eigen::Quaterniond q( Tsymm.linear() );
    viewer->addCube( Tsymm.translation().cast<float>(),
		     q.cast<float>(),
		     (float)2*Bb(0),  (float)2*Bb(1),  (float)2*Bb(2),
		     cn);
*/
  }

  input.close();
  gOutput.close();
/*
  // Visualize
  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
 */ 
}


/**
 * @function downsampling
 */
pcl::PointCloud<PointT>::Ptr downsampling( const pcl::PointCloud<PointT>::Ptr &_input,
						  const double &_voxelSize ) {

  pcl::PointCloud<PointT>::Ptr cloud_downsampled( new pcl::PointCloud<PointT>() );
  
  // Create the filtering object
  pcl::VoxelGrid< PointT > downsampler;
  // Set input cloud
  downsampler.setInputCloud( _input );
  // Set size of voxel
  downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
  // Downsample
  downsampler.filter( *cloud_downsampled );

  return cloud_downsampled;
}

/**
 * @function saveParams
 */
void saveParams( std::ofstream &_output,
		 const SQ_parameters &_par,
		 double _t,
		 double _eg, double _er, 
                 double _e_e1, double _e_e2,
                 double _e_v ) {
  
  _output << _par.dim[0] << " " << _par.dim[1] << " " << _par.dim[2] << " "
	  << _par.e[0] << " " << _par.e[1] << " "
	  << _par.trans[0] << " " << _par.trans[1] << " " << _par.trans[2] << " "
	  << _par.rot[0] << " " << _par.rot[1] << " " << _par.rot[2]
	  << " " << _t << " "<< _eg << " " << _er << " " << _e_e1 << "  " 
          << _e_e2 << " "<< _e_v << std::endl;
}

