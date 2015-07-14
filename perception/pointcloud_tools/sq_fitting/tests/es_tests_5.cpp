/**
 * @file test_cases_generation_base.cpp
 * @brief Generate data to compare downsampling effect
 *@brief By default it does not start from gdD downsampling (not from zero)
 */
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <random>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string>

#include <SQ_utils.h>
#include <evaluated_eqs.h>

#include <future>
#include <thread>

// Global variables
double gnD = 7;
double gdD = 0.005;
double gD = 0;


// Variables that user can set as input
std::string gFilename;

// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize );

/**
 * @function beta
 */
double beta( double z,double w) {
  double gz, gw, gzw;
  
  gz = tgamma(z);
  gw = tgamma(w);
  gzw = tgamma(z+w);
  return  gz*gw/gzw;
}

/**
 * @function volSQ
 */
double volSQ( double a, double b, double c, double e1, double e2 ) {
  return 2*a*b*c*e1*e2*beta(e1*0.5, e1+1)*beta(e2*0.5, e2*0.5+1);
}

/**
 * @brief Structure used to store output lines
 */
struct output_sq{
  SQ_parameters par;
  double t;
  double er_g, er_r;
  double er_v;
  double er_e1;
  double er_e2;
};


// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize );
void saveParams( std::ofstream &_output, const SQ_parameters &_par, double _t,
                 double _eg, double _er, 
                 double _e_e1, double _e_e2,
                 double _e_v );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  // Initialize, in case user does not enter values of e1 and e2
  double a, b, c, e1, e2, px,py,pz,ra,pa,ya;

  a = 0.1; b = 0.1; c = 0.1; e1= 1.0; e2 = 1.0; px = 0; py = 0; pz = 0;
  ra = 0; pa = 0; ya = 0;
  gFilename = std::string("omp_result.txt");

  int v;
  while( (v=getopt(argc, argv, "a:b:c:e:f:x:y:z:r:p:q:n:h")) != -1 ) {
    switch(v) {
    case 'h': {
      printf("Executable to evaluate downsampling effect in one single sample \n");
      printf("Usage: ./Executable -a -b -c -e -f -x -y -z -r -p -q -n output_filename.txt \n");
      return 0;
    } break;
    case 'a' : {
      a = atof(optarg);
    } break;
    case 'b' : {
      b = atof(optarg);
    } break;
    case 'c' : {
      c = atof(optarg);
    } break;
    case 'x' : {
      px = atof(optarg);
    } break;
    case 'y' : {
      py = atof(optarg);
    } break;
    case 'z' : {
      pz = atof(optarg);
    } break;
    case 'r' : {
      ra = atof(optarg);
    } break;
    case 'p' : {
      pa = atof(optarg);
    } break;
    case 'q' : {
      ya = atof(optarg);
    } break;
    case 'e' : {
      e1 = atof(optarg);
    } break;
    case 'f' : {
      e2 = atof(optarg);
    } break;
    case 'n' : {
      gFilename.assign( optarg );
    } break;
    } // switch end
  }

  
  struct timespec start, finish;
  double dt;
  clock_gettime(CLOCK_MONOTONIC, &start);
      
  std::ofstream output( gFilename.c_str(), std::ofstream::out );

  SQ_parameters par, par_res;
  evaluated_sqs es;

  gD = 0;
  
   // Create perfect pointcloud
   pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() ); 
   pcl::PointCloud<pcl::PointXYZ>::Ptr down( new pcl::PointCloud<pcl::PointXYZ>() );
    
  // Dimensions
  par.dim[0] = a; par.dim[1] = b; par.dim[2] = c; 
  // Translation
  par.trans[0] = px; par.trans[1] = py;  par.trans[2] = pz;
  // Rotation
  par.rot[0] = ra; par.rot[1] = pa;  par.rot[2] = ya; 

  // E parameters
  par.e[0] = e1; par.e[1] = e2;

  // Store original data
  output_sq base;
  base.par = par;
  base.er_r = 0; base.er_g = 0; base.t = 0; base.er_v = 0;
  base.er_e1 = 0; base.er_e2 = 0;
  
  saveParams( output, base.par, 0, 0, 0, 0, 0, 0 );
  
  // 1. Generate clean pointcloud
  input = sampleSQ_uniform( par );

  // Loop
  double xer_e1, xer_e2, vc, vr, xer_v;
  struct timespec ts, tf;
  double elapsed;
  double er_g, er_r, er_d;
  
  for( int i = 1; i < gnD; ++i ) {
    
    gD = 0 + gdD*i;
    if( gD == 0 ) { down = input; }
    else { down = downsampling( input, gD ); }    

    // Try it out with methods
    
    clock_gettime(CLOCK_MONOTONIC, &ts);    
    es.minimize( down, par_res, er_g, er_r, er_d, SQ_FX_RADIAL );
    clock_gettime(CLOCK_MONOTONIC, &tf);
    error_metric( par_res,input, er_g, er_r, er_d );
    
    elapsed = (tf.tv_sec - ts.tv_sec);
    elapsed += (tf.tv_nsec - ts.tv_nsec) / 1000000000.0;
    
    xer_e1 = fabs(base.par.e[0] - par_res.e[0]);
    xer_e2 = fabs(base.par.e[1] - par_res.e[1]);

    vc =  volSQ(par_res.dim[0], par_res.dim[1], par_res.dim[2],     par_res.e[0], par_res.e[1]);
    vr = volSQ(base.par.dim[0],base.par.dim[1],base.par.dim[2],  base.par.e[0], base.par.e[1] );
    xer_v = (vc - vr)/vr*100.0;

    output << down->points.size() << std::endl;   
    saveParams( output, par_res, elapsed, er_g, er_r, xer_e1, xer_e2, xer_v);  
    
  }
  
  clock_gettime(CLOCK_MONOTONIC, &finish);
  
  dt = (finish.tv_sec - start.tv_sec);
  dt += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
  printf("* Total time: %f \n", dt );
  
  output.close();
  return 0;
}




/**
 * @function downsampling
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled( new pcl::PointCloud<pcl::PointXYZ>() );
  
  // Create the filtering object
  pcl::VoxelGrid< pcl::PointXYZ > downsampler;
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

