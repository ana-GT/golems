/**
 * @function SQ_utils_SE_test.cpp
 */
#include <pcl/io/pcd_io.h>
#include <SQ_utils.h>
#include <SQ_deformations.h>
#include <analytic_equations.h>
#include <sq_fitting/SQ_fitter.h>

int main( int argc, char* argv[] ) {

  double a1 = 0.15;
  double a2 = 0.15;
  double a3 = 0.15;
  double e1 = 0.1;
  double e2 = 1.0;
  int N = 50;
  double t = 0.5; // <0,1>
  int v;
  double Rfactor, R;
  Rfactor = 1.04;

  while( (v=getopt(argc, argv, "n:a:b:c:e:f:t:r:")) != -1 ) {
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
    case 't' : {
      t = atof(optarg);
    } break;
    case 'r' : {
      Rfactor = atof(optarg);
    }
    } // switch end
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
 
  std::cout << "a1: "<< a1 << " a2: "<< a2 << " a3: "<< a3 << " e1: "<< e1<<" e2: "<< e2 <<" t: "<< t <<  std::endl;
  std::cout << " num points: "<< N << std::endl;
    cloud = sampleSQ_uniform( a1, a2, a3, e1, e2, N );
  
  std::cout << "Cloud size: "<< cloud->points.size() << std::endl;
  pcl::io::savePCDFileASCII ( "original.pcd", *cloud );

  // My deformation...
  typename pcl::PointCloud<pcl::PointXYZ>::iterator pm;
  int im;
  pcl::PointCloud<pcl::PointXYZ>::Ptr myDef( new pcl::PointCloud<pcl::PointXYZ>() );
  myDef->points.resize( cloud->points.size() );
  myDef->width = 1;
  myDef->height = cloud->points.size();
  double xi, yi, zi;
  R = a3*Rfactor; 
  for( pm = cloud->begin(), im = 0; pm != cloud->end(); ++pm, ++im ) {
    xi = (*pm).x; yi = (*pm).y; zi = (*pm).z;    
    myDef->points[im].x = xi;
    myDef->points[im].y = yi - R*(1.0-cos(zi/R) );
    myDef->points[im].z = R*sin(zi/R);
  }
  pcl::io::savePCDFileASCII ( "myDef.pcd", *myDef );
  
  

  // Tampered
  pcl::PointCloud<pcl::PointXYZ>::Ptr tampered;
  SQ_deformations sqd;
  tampered = sqd.linear_tampering( a1, a2, a3, e1, e2, t );
  pcl::io::savePCDFileASCII ( "tampered.pcd", *tampered );

// Check levmar tampering
  int m = 12;
  int n = tampered->points.size();
 double* p = new double[m];
  double* x = new double[n]; 
  
 // Fill p
   p[0] = a1; p[1]= a2; p[2] = a3; p[3] = e1; p[4] = e2;
   p[5] = 0; p[6] = 0; p[7] = 0; p[8] = 0; p[9] = 0; p[10] = 0;
   p[11]= t;

 // Fill data
    struct levmar_data data;
    data.x = new double[n];
    data.y = new double[n];
    data.z = new double[n];
    data.num = n;

    int i; int ret;
    typename pcl::PointCloud<pcl::PointXYZ>::iterator pit;
    for( pit = tampered->begin(), i = 0; pit != tampered->end(); ++pit, ++i ) {
	data.x[i] = (*pit).x;
	data.y[i] = (*pit).y;
	data.z[i] = (*pit).z;
    }

  levmar_tampering_fx( p, x,  m, n, (void*)&data ); 

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
    recovered = sqd.linear_tampering( output.dim[0], output.dim[1],
				      output.dim[2], output.e[0],
				      output.e[1], output.tamp );
    pcl::io::savePCDFileASCII ( "recovered.pcd", *recovered );



  } else {
    printf("CRAP, did not fit! \n");

  }

  return 0;
}
