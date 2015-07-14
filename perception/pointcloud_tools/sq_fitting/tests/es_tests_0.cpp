/**
 * @file es_tests_0.cpp
 * @brief Test an individual case, values entered by user (a,b,c,e1,e2,px,py,pz,ra,pa,ya)
 */
#include "evaluated_eqs.h"
#include <SQ_utils.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

char* fx_names[6] = { "Radial", "Solina", "Ichim", "Chevalier", "F5", "F6"};
int fx_sq[6] = {SQ_FX_RADIAL, SQ_FX_SOLINA, SQ_FX_ICHIM, SQ_FX_CHEVALIER, SQ_FX_5, SQ_FX_6 };
const double gDev = 0.0025;

// Functions to get partial and noisy versions of original pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampling( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_input,
						  const double &_voxelSize );
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud );
pcl::PointCloud<pcl::PointXYZ>::Ptr get_noisy( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
					       const double &_dev );
double getRand( const double &_minVal, const double &_maxVal );

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




// Global variables to generate noise
std::random_device rd;
std::mt19937 gen(rd());
double gD;

/**
 * @function main
 */
int main( int argc, char*argv[] ) {

  double a1 = 0.15;
  double a2 = 0.03;
  double a3 = 0.06; 
  double e1 = 0.5;
  double e2 = 0.75; 
  double px = 0.1; double py = 0.2; double pz = 0.4;
  double ra = 0.4; double pa = -0.3; double ya = 0.1;
  gD = 0.0; // Downsampling
  int N = 50;
  int v;
  
  while( (v=getopt(argc, argv, "n:a:b:c:e:f:x:y:z:r:p:d:h")) != -1 ) {
    switch(v) { 

    case 'h' : {
      printf("Syntax: ./executable -a -b -c -e -f -x -y -z -r -p -d DOWNSAMPLE_STEP \n");
      return 0;
    } break;
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
    case 'd' : {
     gD = atof(optarg);
    } break;
    } // switch end
  }
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr input( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::PointCloud<pcl::PointXYZ>::Ptr down( new pcl::PointCloud<pcl::PointXYZ>() );

  SQ_parameters par, base;
  double er_g, er_r, er_d, er_v;
  double vr, vc;
  par.dim[0] = a1; par.dim[1] = a2; par.dim[2] = a3;
  par.e[0] = e1; par.e[1] = e2;
  par.trans[0] = px; par.trans[1] = py; par.trans[2] = pz;
  par.rot[0] = ra; par.rot[1] = pa; par.rot[2] = ya;
  base = par;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> testCloud(4);
  
  input = sampleSQ_uniform( par );
  if( gD == 0 ) { down = input; }
  else { down = downsampling( input, gD ); }
  testCloud[0] = down;    
  testCloud[1]= get_noisy( testCloud[0], gDev );
  testCloud[2] = cut_cloud( testCloud[0] );
  testCloud[3] = cut_cloud( testCloud[1] );

  
  printf("Size of real input: %d \n", input->points.size() );
  printf("Size of input to minimization: %d \n", down->points.size() );
  
  clock_t ts, tf; double dt;
  evaluated_sqs es;

  for( int i = 0; i < 6; ++i ) {
    printf( "Function: %s \n", fx_names[i] );
    for( int j = 0; j < 4; ++j ) {
      ts = clock();
      es.minimize( testCloud[j], par, er_g, er_r, er_d, fx_sq[i] );
      tf = clock();
      dt = (tf-ts) / (double) CLOCKS_PER_SEC;

      error_metric( par, input, er_g, er_r, er_d );
      vc =  volSQ(par.dim[0], par.dim[1], par.dim[2], par.e[0], par.e[1]);
      vr = volSQ(base.dim[0],base.dim[1],base.dim[2],  base.e[0], base.e[1] );
      er_v = (vc - vr)/vr*100.0;
      
      printf("Dim: %f %f %f \t e: %f %f \t t: %f \t er_g: %f er_r: %f er_d: %f er_v: %f \% \n",
	     par.dim[0], par.dim[1], par.dim[2],
	     par.e[0], par.e[1], dt,
	     er_g, er_r, er_d, er_v);
      
    }
  }
   
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
 * @function cut_cloud
 * @brief Slice cloud by half, leaving only the "visible"points (points towards the eye point (0,0,0))
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr output( new pcl::PointCloud<pcl::PointXYZ>() );
  
  // Get eigenvectors and centroid of pointcloud
  Eigen::Vector4d c;
  pcl::compute3DCentroid( *_cloud, c );
  
  // Get the normal w.r.t. the centroid
  Eigen::Vector3d N; N = Eigen::Vector3d(0,0,0) - Eigen::Vector3d(c(0), c(1), c(2) );
  double d;
  
  // Plane equation n(0)x + n(1)y + n(2)z + d = 0. Find d using the centroid
  d = -( N(0)*c(0) + N(1)*c(1) + N(2)*c(2) );
  
  // Cut it
  for( pcl::PointCloud<pcl::PointXYZ>::iterator it = _cloud->begin();
	   it != _cloud->end(); it++ ) {
    
    if( (N(0)*((*it).x) + N(1)*((*it).y) + N(2)*((*it).z) + d) > 0 ) {
      pcl::PointXYZ p;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr get_noisy( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
					       const double &_dev ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr output( new pcl::PointCloud<pcl::PointXYZ>() );
  std::normal_distribution<double> d(0,_dev);
  
  // Make it dirty
  for( pcl::PointCloud<pcl::PointXYZ>::iterator it = _cloud->begin();
       it != _cloud->end(); it++ ) {
    pcl::PointXYZ p;
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

  return _minVal + (_maxVal - _minVal)*((double)rand() / RAND_MAX);
  
}
