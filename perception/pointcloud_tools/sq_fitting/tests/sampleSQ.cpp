/**
 * @file sampleSQ.cpp
 * @brief Generate pointcloud with Super Quadric sampled
 * @author A. Huaman Quispe
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <unistd.h>
#include <stdlib.h> 

#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>
#include "perception/pointcloud_tools/refresher_utils/Refresher_utils.h"

#include "SQ_utils.h"

void printHelp();

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

    // Parse the input arguments
    int v;
    double a,b,c;
    double e1,e2;
    int n;
    double x,y,z, ra,pa,ya;
    std::string filename;

    // Initialize them
    a = 0.5; b = 0.5; c = 0.5;
    e1 = 0.5; e2 = 0.5;
    x = 0; y = 0; z = 0;
    ra = 0.0; pa = 0; ya = 0;
    n = 100; 
    filename = std::string("sampleSQ_output.pcd");

    opterr = 0;
    while( (v = getopt(argc, argv, "N:a:b:c:x:y:z:R:P:Y:e:f:n:hn")) != -1 ) {
	
	switch(v) {
	case 'N': {
	    filename = std::string( optarg );
	} break;
	case 'a': {
	    a = atof( optarg );
	} break;
	case 'b': {
	    b = atof( optarg );
	} break;
	case 'c': {
	    c = atof( optarg );
	} break;
	case 'x': {
	    x = atof( optarg );
	} break;
	case 'y': {
	    y = atof( optarg );
	} break;
	case 'z': {
	    z = atof( optarg );
	} break;
	case 'R': {
	    ra = atof( optarg );
	} break;
	case 'P': {
	    pa = atof( optarg );
	} break;
	case 'Y': {
	    ya = atof( optarg );
	} break;
	case 'e': {
	    e1 = atof( optarg );
	} break;
	case 'f': {
	    e2 = atof( optarg );
	} break;
	case 'n': {
            n = atoi( optarg );
        } break;
	case 'h': {
	  printHelp();
	  return 1;
	} break;	  
    case '?': {
		printHelp();	
		return 1;
	} break;
	} // end of switch

    } // end of while


    // Sample SQ uniform
    SQ_parameters par; 
    par.dim[0] = a; par.dim[1] = b; par.dim[2] = c;
    par.e[0] = e1; par.e[1] = e2;
    par.trans[0] = x; par.trans[1] = y; par.trans[2] = z;
    par.rot[0] = ra; par.rot[1] = pa; par.rot[2] = ya;
    
    // See values
    std::cout <<"\t ** Sampled SQ Information: **"<< std::endl;
    printParamsInfo( par );

    // Generate samples
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
    cloud = sampleSQ_uniform( par );
    printf("Sampe uniform pn \n");
    pcl::PointCloud<pcl::PointNormal>::Ptr pn( new pcl::PointCloud<pcl::PointNormal>() );
    sampleSQ_uniform_pn( a, b, c, e1, e2, n, pn );
    
    pcl::io::savePCDFileASCII( "pntest.pcd", *pn );        
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(pn);
    pcl::PolygonMesh mesh;
    poisson.reconstruct( mesh );
    pcl::io::saveOBJFile("pntest.obj", mesh);
    
    // Save
    pcl::io::savePCDFileASCII( filename, *cloud );        
    std::cout <<"\t [*] Saved "<< filename  << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ct( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::Normal>::Ptr nt( new pcl::PointCloud<pcl::Normal>() );

    for( int i = 0; i < pn->points.size(); ++i ) {
      pcl::PointXYZ a; pcl::Normal b;
      a.x = pn->points[i].x;
      a.y = pn->points[i].y;
      a.z = pn->points[i].z;

      b.normal_x = pn->points[i].normal_x;
      b.normal_y = pn->points[i].normal_y;
      b.normal_z = pn->points[i].normal_z;
      
      ct->points.push_back(a);
      nt->points.push_back(b);
    }

    Eigen::Vector3i color;
    pointcloudToMesh( ct, "testOldMethod.off", color );
    
    /*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ> (ct, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (ct, nt, 1, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
      {
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
    */
    return 0;
}

/**
 * @function printHelp
 */
void printHelp() {
  std::cout <<" Syntax: ./sampleSQ  -a A -b B -c C -e E1 -f E2 -x X -y Y -z Z -R roll -P pitch -Y yaw -n NumIter -N filename.pcd "<< std::endl;
  return;
}
