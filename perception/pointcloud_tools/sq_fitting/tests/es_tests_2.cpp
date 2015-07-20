/**
 * @file es_tests_2.cpp
 * @brief Test a set of random cases for noise and partiality using threads
 */
#include "evaluated_eqs.h"
#include <SQ_utils.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

#include <future>
#include <thread>

typedef pcl::PointXYZ PointT;

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


// Global variables
const int gnF = 5;
char* fx_names[gnF] = { "Radial", "Solina", "Ichim", "Chevalier", "F5"};
int fx_sq[gnF] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5};
const double gDev = 0.0025;

const int gNum_threads = 7;
int gT = 50;
std::string gFilename;
double gD = 0.015;

// Global variables to generate noise
std::random_device rd;
std::mt19937 gen(rd());

// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<PointT>::Ptr downsampling( const pcl::PointCloud<PointT>::Ptr &_input,
						  const double &_voxelSize );
pcl::PointCloud<PointT>::Ptr cut_cloud( const pcl::PointCloud<PointT>::Ptr &_cloud );
pcl::PointCloud<PointT>::Ptr get_noisy( const pcl::PointCloud<PointT>::Ptr &_cloud,
					       const double &_dev );
double getRand( const double &_minVal, const double &_maxVal );
double beta( double z,double w);
double volSQ( double a, double b, double c, double e1, double e2 );
void saveParams( std::ofstream &_output, const SQ_parameters &_par, double _t,
                 double _eg, double _er, 
                 double _e_e1, double _e_e2,
                 double _e_v );
std::vector<output_sq> createCases( int _id );
std::vector<output_sq> createCase();



/**
 * @function main
 */
int main( int argc, char*argv[] ) {


  /* initialize random seed: */
  srand (time(NULL));
  
  // Initialize, in case user does not enter values of e1 and e2
  gFilename = std::string("test_2_result.txt");
  
  int v;  
  while( (v=getopt(argc, argv, "t:n:h")) != -1 ) {
    switch(v) { 
    case 'n' : {
      gFilename.assign(optarg);
    } break;
    case 't' : {
      gT = atoi(optarg);
    } break;
    case 'h' : {
      printf("Executable to save T randomized runs to test noise and partial view \n");
      printf("Usage: ./executable -t T -n output_filename.txt \n");
      return 0;
    } break;
    } // switch end
  }

  struct timespec start, finish;
  double elapsed;
  clock_gettime(CLOCK_MONOTONIC, &start);
  
  srand(time(NULL));
      
  std::ofstream output( gFilename.c_str(), std::ofstream::out );
  output << gT << std::endl;
  
  // Launch threads
  std::vector<std::future< std::vector<output_sq> > > futures;
  for( int i = 0; i < gNum_threads; ++i ) {
    futures.push_back( std::async( std::launch::async, &createCases, i) );
  }

  for( size_t i = 0; i < futures.size(); ++i ) {
    futures[i].wait();
  }

  for( size_t i = 0; i < futures.size(); ++i ) {
    std::vector<output_sq> s;
    s = futures[i].get();
    for( int m = 0; m < s.size(); ++m ) {
      saveParams( output, s[m].par, s[m].t, s[m].er_g, s[m].er_r,
		  s[m].er_e1, s[m].er_e2, s[m].er_v );
    }
  }  
  
  clock_gettime(CLOCK_MONOTONIC, &finish);
  
  elapsed = (finish.tv_sec - start.tv_sec);
  elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
  printf("* Total time: %f \n", elapsed );
  
  output.close();
  return 0;

}

/**
 * @function createCases
 * @brief Create cases per each thread
 */
std::vector<output_sq> createCases(int _id ) {

  std::vector<output_sq> ss;
  std::vector<output_sq> si;
  for( int i = _id; i < gT; i = i + gNum_threads ) {
    si = createCase();
    for( int j = 0; j < si.size(); ++j ) {
      ss.push_back( si[j] );
    }    
  }
  
  return ss;
}

/**
 * @function createCase
 * @brief Create individual case (base + 6*4 tests)
 */
std::vector<output_sq> createCase() {

  std::vector<output_sq> output;
  pcl::PointCloud<PointT>::Ptr input( new pcl::PointCloud<PointT>() );
  pcl::PointCloud<PointT>::Ptr down( new pcl::PointCloud<PointT>() );
  std::vector<pcl::PointCloud<PointT>::Ptr> testCloud(4);
  
  SQ_parameters par;
  double er_g, er_r, er_d;
  evaluated_sqs es;
    
  // Dimensions
  par.dim[0] = getRand(0.025, 0.39);
  par.dim[1] = getRand(0.025, 0.39);
  par.dim[2] = getRand(0.025, 0.39);
  
  // Translations
  par.trans[0] = getRand(-0.8, 0.8);
  par.trans[1] = getRand(-0.8, 0.8);
  par.trans[2] = getRand(0.3, 1.4);
  
  // Rotation
  par.rot[0] = getRand(-M_PI, M_PI);
  par.rot[1] = getRand(-M_PI, M_PI); 
  par.rot[2] = getRand(-M_PI, M_PI); 

  // E parameters
  par.e[0] = getRand(0.1,1.9);
  par.e[1] = getRand(0.1,1.9);


  // Store original data
  output_sq base;
  base.par = par;
  base.er_r = 0; base.er_g = 0; base.t = 0; base.er_v = 0;
  base.er_e1 = 0; base.er_e2 = 0;
  output.push_back( base );
    
  // 1. Generate clean pointcloud
  input = sampleSQ_uniform( par );

  struct timespec ts, tf;
  double elapsed;
  double d;
  double vr, vc;
  output_sq oi;

  if( gD == 0 ) { down = input; }
  else { down = downsampling( input, gD ); }
  testCloud[0] = down;    
  testCloud[1]= get_noisy( testCloud[0], gDev );
  testCloud[2] = cut_cloud( testCloud[0] );
  testCloud[3] = cut_cloud( testCloud[1] );

  for( int i = 0; i < gnF; ++i ) {
    for( int j = 0; j < 4; ++j ) {
      
      clock_gettime(CLOCK_MONOTONIC, &ts);    
      es.minimize( testCloud[j], par, er_g, er_r, er_d, fx_sq[i] );
      clock_gettime(CLOCK_MONOTONIC, &tf);
      error_metric<PointT>( par,input, er_g, er_r, er_d );
      
      elapsed = (tf.tv_sec - ts.tv_sec);
      elapsed += (tf.tv_nsec - ts.tv_nsec) / 1000000000.0;
     
      oi.par = par;
      oi.er_g = er_g; oi.er_r = er_r; oi.t = elapsed;
      oi.er_e1 = fabs(base.par.e[0] - par.e[0]);
      oi.er_e2 = fabs(base.par.e[1] - par.e[1]);
      
      vc =  volSQ(par.dim[0], par.dim[1], par.dim[2], par.e[0], par.e[1]);
      vr = volSQ(base.par.dim[0],base.par.dim[1],base.par.dim[2],  base.par.e[0], base.par.e[1] );
      oi.er_v = (vc - vr)/vr*100.0;
    
      output.push_back( oi );    
      
    }
  }
    
  return output;
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
 * @function cut_cloud
 * @brief Slice cloud by half, leaving only the "visible"points (points towards the eye point (0,0,0))
 */
pcl::PointCloud<PointT>::Ptr cut_cloud( const pcl::PointCloud<PointT>::Ptr &_cloud ) {
  
  pcl::PointCloud<PointT>::Ptr output( new pcl::PointCloud<PointT>() );
  
  // Get eigenvectors and centroid of pointcloud
  Eigen::Vector4d c;
  pcl::compute3DCentroid( *_cloud, c );
  
  // Get the normal w.r.t. the centroid
  Eigen::Vector3d N; N = Eigen::Vector3d(0,0,0) - Eigen::Vector3d(c(0), c(1), c(2) );
  double d;
  
  // Plane equation n(0)x + n(1)y + n(2)z + d = 0. Find d using the centroid
  d = -( N(0)*c(0) + N(1)*c(1) + N(2)*c(2) );
  
  // Cut it
  for( pcl::PointCloud<PointT>::iterator it = _cloud->begin();
	   it != _cloud->end(); it++ ) {
    
    if( (N(0)*((*it).x) + N(1)*((*it).y) + N(2)*((*it).z) + d) > 0 ) {
      PointT p;
      p = *it;
      output->points.push_back( p );
    }
    
  }
  
  output->height = 1; output->width = output->points.size();
  
  return output;  
}

/**
 * @function get_noisy
 */
pcl::PointCloud<PointT>::Ptr get_noisy( const pcl::PointCloud<PointT>::Ptr &_cloud,
					       const double &_dev ) {
  
  pcl::PointCloud<PointT>::Ptr output( new pcl::PointCloud<PointT>() );
  std::normal_distribution<double> d(0,_dev);
  
  // Make it dirty
  for( pcl::PointCloud<PointT>::iterator it = _cloud->begin();
       it != _cloud->end(); it++ ) {
    PointT p;
    p.x = (*it).x + d(gen);
    p.y = (*it).y + d(gen);
    p.z = (*it).z + d(gen);
    output->points.push_back( p );
  }
  
  output->height = 1; output->width = output->points.size();
  return output;
}

/**
 * @function getRand
 */
double getRand( const double &_minVal, const double &_maxVal ) {

  return _minVal + (_maxVal - _minVal)*((double)rand() / (double)RAND_MAX);
  
}


double beta( double z,double w) {
  double gz, gw, gzw;
  
  gz = tgamma(z);
  gw = tgamma(w);
  gzw = tgamma(z+w);
  return  gz*gw/gzw;
}

double volSQ( double a, double b, double c, double e1, double e2 ) {
  return 2*a*b*c*e1*e2*beta(e1*0.5, e1+1)*beta(e2*0.5, e2*0.5+1);
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
