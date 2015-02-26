/**
 * @file Refresher_utils.cpp
 */
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

#include <pcl/surface/concave_hull.h>
#include <iostream>
#include <fstream>

#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/MeshShape.h>
#include <dart/simulation/World.h>
#include <dart/dynamics/FreeJoint.h>

#include <refresher_utils/Refresher_utils.h>
#include <sq_fitting/SQ_parameters.h>
#include <sq_fitting/SQ_fitter.h>

void downsample( const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud,
		 const double &_voxelSize,
		 pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud_downsampled ) {
    
    // Create the filtering object
    pcl::VoxelGrid< pcl::PointXYZ > downsampler;
    // Set input cloud
    downsampler.setInputCloud( _cloud );
    // Set size of voxel
    downsampler.setLeafSize( _voxelSize, _voxelSize, _voxelSize );
    // Downsample
    downsampler.filter( *_cloud_downsampled );
    
}

bool pointcloudToMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
		       std::string _filename,
		       Eigen::Vector3i _color ) {

    pcl::ConcaveHull<pcl::PointXYZ> hull;
    pcl::PointCloud<pcl::PointXYZ> points;
    std::vector<pcl::Vertices> faces;

    // Call reconstruct
    hull.setInputCloud( _cloud );
    hull.setAlpha (0.01);
    hull.reconstruct( points, faces );

    // Save in a mesh file
    saveMeshFile( points, faces, _filename, _color );

    return true;
}

/**
 * @function saveMeshFile
 */
bool saveMeshFile( pcl::PointCloud<pcl::PointXYZ> _points,
		   std::vector<pcl::Vertices> _faces,
		   std::string _filename,
		   Eigen::Vector3i _color ) {

    // Open file
    std::ofstream file;
    file.open( _filename.c_str() );

    // Write off keyword
    file << "OFF" << "\n";

    // Write number of vertices and number of faces
    file << _points.size() << " " << _faces.size() << " "<<0<<" \n";

    // Write the n vertices
    pcl::PointXYZ p;
    for( int i = 0; i < _points.size(); ++i ) {	
	p = _points.points[i];
	file << p.x << " "<< p.y << " "<< p.z << "\n";
    }
    
    // Write the m faces
    for( int i = 0; i < _faces.size(); ++i ) {
	file << _faces[i].vertices.size() << " ";
	for( int j = 0; j < _faces[i].vertices.size(); ++j ) {
	    file << _faces[i].vertices[j] << " ";
	}
	file << " 3 "<<" "<< _color.transpose() << "\n";
    }

    // Close
    file.close();
}

/*
bool SQToMesh( SQ_parameters _param,
	       std::string _filename ) {

    // Sample the SQ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
    cloud = sampleSQ_uniform( _param );

    return true;
}
*/

/**
 * @function pointcloudToBB
 */
void pointcloudToBB( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
		     double _dim[3],
		     Eigen::Isometry3d &_Tf ) {


    // 1. Compute the bounding box center
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid( *_cloud, centroid );
    
    // 2. Compute main axis orientations
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud( _cloud );
    Eigen::Vector3f eigVal = pca.getEigenValues();
    Eigen::Matrix3f eigVec = pca.getEigenVectors();
    // Make sure 3 vectors are normal w.r.t. each other
    eigVec.col(2) = eigVec.col(0); // Z
    Eigen::Vector3f v3 = (eigVec.col(1)).cross( eigVec.col(2) );
    eigVec.col(0) = v3; 
    
    /*
      Eigen::Vector3f v3 = (eigVec.col(0)).cross( eigVec.col(1) );
      eigVec.col(2) = v3;
    */
    for( int j = 0; j < 3; ++j ) {
	for( int k = 0; k < 3; ++k ) {
	    _Tf.linear()(j,k) = (double)eigVec(j,k);	
	}
    }
    _Tf.translation() << centroid(0), centroid(1), centroid(2);

    // Transform _cloud
    Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
    transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
    transf.block(0,0,3,3) = eigVec;
    
    Eigen::Matrix4f tinv; tinv = transf.inverse();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::transformPointCloud( *_cloud, *cloud_temp, tinv );
    
    // Get maximum and minimum
    pcl::PointXYZ minPt; pcl::PointXYZ maxPt;
    pcl::getMinMax3D( *cloud_temp, minPt, maxPt );
    
    _dim[0] = ( maxPt.x - minPt.x );
    _dim[1] = ( maxPt.y - minPt.y );
    _dim[2] = ( maxPt.z - minPt.z ); 

}


