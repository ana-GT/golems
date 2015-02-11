/**
 * @file crichton_eye_v2
 * @brief Reads a scene pointcloud, segments and fit the type of primitive you want to use to represent the objects
 * @author A. Huaman Q.
 */
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <Eigen/Core>
#include <stdint.h>
#include "tabletop_segmentation/tabletop_segmentation.h"

// SQ, BB
#include "refresher_utils/Refresher_utils.h"
#include "sq_fitting/SQ_utils.h"
#include "sq_fitting/SQ_fitter.h"
#include "tabletop_symmetry/mindGapper.h"

/**************************/
/** GLOBAL VARIABLES      */
/**************************/
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > gClusters;
std::vector<Eigen::Vector3d> gColors;
std::vector<double> mTableCoeffs;

int mType;
std::string mFilename;
boost::shared_ptr<pcl::visualization::PCLVisualizer> gViewer;

/***********************/
/** FUNCTIONS          */
/***********************/
void process( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud );
boost::shared_ptr<pcl::visualization::PCLVisualizer> clusterViewer ( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > clouds,
								     std::vector<Eigen::Vector3d> colors );
void printHelp( char* _argv0 );
void fit_BB( std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > _clusters );
void fit_SQ( std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > _clusters );

enum {
    BOUNDING_BOX = 0,
    SUPERQUADRIC = 1
};


/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  int c;
  while( (c=getopt(argc,argv,"ht:")) != -1 ) {
    switch(c) {
    case 'h': { printHelp( argv[0] ); return 1; } break;
    case 't': { mType = atoi(optarg); } break; 	    
    }
  }

  // Read pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGBA>() );    
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( mFilename.c_str(), *cloud) == -1) {
    printf("\t * Couldn't read file %s \n", mFilename.c_str() );
    return 1;
  }
  
  // Segment it
  process( cloud );
  
  // Fit it with the chosen method
  switch( mType ) {
  case BOUNDING_BOX: { fit_BB( gClusters ); } break;
  case SUPERQUADRIC: { fit_SQ( gClusters ); } break;
  case REVOLUTE: { fit_REV( gClusters ); } break;
  case EXTRUSION: { fit_EXT( gClusters ); } break;
    };
     

    /*
    gViewer = clusterViewer( gClusters, gColors );
        
    while( !gViewer->wasStopped() ) {
	gViewer->spinOnce(100);
	boost::this_thread::sleep( boost::posix_time::seconds(1) );
    }
    */  
    return 0;
}

/**
 * @function fit_BB
 * @brief 
 */
void fit_BB( std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > _clusters ) {
    
    double dim[3]; Eigen::Isometry3d Tf;

    for( int i = 0; i < _clusters.size(); ++i ) {
	printf("\t * Fitting box for cluster %d \n", i );

	pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
	p->points.resize( _clusters[i]->points.size() );
	for(int j = 0; j < _clusters[i]->points.size(); ++j ) {
	    p->points[j].x = _clusters[i]->points[j].x;
	    p->points[j].y = _clusters[i]->points[j].y;
	    p->points[j].z = _clusters[i]->points[j].z;
	}
	p->width = 1; p->height = p->points.size();

	// Get Bounding Box
	pointcloudToBB( p, dim, Tf );
	// Store (cube)
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts( new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::PointCloud<pcl::PointXYZ>::Ptr final( new pcl::PointCloud<pcl::PointXYZ>() );

	pts = sampleSQ_uniform( dim[0]/2, dim[1]/2, dim[2]/2, 0.1, 0.1 );
	pcl::transformPointCloud( *pts, *final, Tf.matrix() );

	char name[50];
	sprintf( name, "bb_%d.pcd", i);
	pcl::io::savePCDFileASCII(name, *final );
    }


}

/**
 * @function fit_SQ
 */
void fit_SQ( std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > _clusters ) {

    // Generate a mirror version of the pointcloud
    mindGapper<pcl::PointXYZRGBA> mg;
    mg.setTablePlane( mTableCoeffs );
    mg.setFittingParams();
    mg.setDeviceParams();

    // Fit pointcloud to superquadric
    SQ_fitter< pcl::PointXYZ > fitter;
    for( int i = 0; i < _clusters.size(); ++i ) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr completed( new pcl::PointCloud<pcl::PointXYZRGBA>() );
	*completed = *_clusters[i];
	mg.reset();
	mg.complete( completed );
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr p( new pcl::PointCloud<pcl::PointXYZ>() );
	p->points.resize( completed->points.size() );
	for(int j = 0; j < completed->points.size(); ++j ) {
	    p->points[j].x = completed->points[j].x;
	    p->points[j].y = completed->points[j].y;
	    p->points[j].z = completed->points[j].z;
	}
	p->width = 1; p->height = p->points.size();
	
	fitter.setInputCloud( p );
	if( fitter.fit( LEVMAR_MINIMIZER, 0.03, 0.005, 5, 0.1 ) ) {
	    
	    SQ_parameters p;
	    fitter.getFinalParams( p );	    
	    pcl::PointCloud<pcl::PointXYZ>::Ptr pts( new pcl::PointCloud<pcl::PointXYZ>() );
	    pts = sampleSQ_uniform( p );

	    char name[50];
	    sprintf( name, "sq_%d.pcd", i);
	    pcl::io::savePCDFileASCII(name, *pts );

	}
    } // end for

}


/**
 * @function process
 */
void process( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud ) {

  // Segment
  TabletopSegmentor<pcl::PointXYZRGBA> tts;

  tts.processCloud( _cloud );
  mTableCoeffs = tts.getTableCoeffs();
  int n = tts.getNumClusters();

  // Set segmented variables
  gClusters.resize(0);
  for( int i = 0; i < n; ++i ) {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster( new pcl::PointCloud<pcl::PointXYZRGBA>() );
      gClusters.push_back( cluster );
  }

  gColors.resize(n);
  


  for( int i = 0; i < n; ++i ) {
     
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster( new pcl::PointCloud<pcl::PointXYZRGBA>() );
      *cluster = tts.getCluster(i);
      gClusters[i] = cluster;
      Eigen::Vector3i def;
      def(0) = rand() % 255; def(1) = rand() % 255; def(2) = rand() % 255;  
      gColors[i]<< (double) def(0) / 255.0, (double) def(1) / 255.0, (double) def(2) / 255.0;
  }
  
}

/**
 * @function printHelp
 */
void printHelp( char* _argv0 ) {
    printf("\t Usage: %s -r FILENAME.pcd -t TYPE \n", _argv0 );
    printf("\t -t 0 : BOUNDING BOX \n");
    printf("\t -t 1 : SUPERQUADRIC \n");
    printf("\t -t 2 : REVOLUTE \n");
    printf("\t -t 3 : EXTRUSION \n");
    printf("\t -h : This help \n");
    printf("\t -r FILENAME.pcd \n");
}


/**
 * @function createViewer
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> clusterViewer ( std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > clouds,
								     std::vector<Eigen::Vector3d> colors ) {
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Cluster Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    
    for( int i = 0; i < clouds.size(); ++i ) {
	
	char name[50];
	sprintf( name, "cloud_%d", i );
	printf("[%d] Cloud size: %d (%d, %d ) \n", i, clouds[i]->points.size(), clouds[i]->width, clouds[i]->height );
	viewer->addPointCloud<pcl::PointXYZRGBA> ( clouds[i], name );
	printf("Step 2 \n");
	viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 
						  colors[i](0), colors[i](1), colors[i](2), name );
	printf("Step 3 \n");
    }
    
    viewer->addCoordinateSystem (1.0, 0);
    viewer->initCameraParameters ();
    return (viewer);
}
