/**
 * @file LocalRecognizer.cpp
 * @brief Implementation of global recognizer 
 */
#include "LocalRecognizer.h"

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>

#include <stdio.h>
#include <iostream>
#include <fstream>


model::model() : 
  points( new pcl::PointCloud<PointType>() ),
  normals( new pcl::PointCloud<NormalType>() ),
  keypoints( new pcl::PointCloud<PointType>() ),
  descriptors( new pcl::PointCloud<DescriptorType>() ) {
}


/**
 * @function LocalRecognizer
 * @brief Constructor
 */
LocalRecognizer::LocalRecognizer() :
  mModel_scene_corrs( new pcl::Correspondences() ) {
  mModels.resize(0);
  mRadius = 0.01f;
  mDescriptor_Radius = 0.02f;
  mCg_size = 0.01f;
  mCg_thresh = 5.0f;
}


/**
 * @function LocalRecognizer
 * @brief Destructor
 */
LocalRecognizer::~LocalRecognizer() {

}


/**
 * @function load 
 * @brief Load json file with info from objects to be recognized 
 */
bool LocalRecognizer::load( std::string _obj_input ) {

  mModels.resize(0);
  
  Json::Value root;
  Json::Reader reader;

  std::ifstream obj_string( _obj_input.c_str(), std::ifstream::binary );
  
  bool parsingSuccessful = reader.parse( obj_string, root );
  if( !parsingSuccessful ) {
    std::cout << "Failed to parse file: \n "<< reader.getFormattedErrorMessages()<<std::endl;
    return false;
  } 
  Json::Value j_objects = root["objects"];
  for( int i = 0; i < j_objects.size(); ++i ) {
    mModels.push_back( loadModel( j_objects[i] ) );
  }

  return true;  
}

/**
 * @function viewModel
 */
void LocalRecognizer::viewModels() {
  pcl::visualization::PCLVisualizer viewer("View Keypoints");
  viewer.addCoordinateSystem(1.0);
 
  for( int i = 0; i < mModels.size(); ++i ) {
    char name[25];
    sprintf( name, "keypoints_%d", i );
    viewer.addPointCloud<PointType>( mModels[i].points, std::string(name) );
  }
  while( !viewer.wasStopped() ) {
    viewer.spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000) );
  }

}

/**
 * @function prepareModels
 * @brief Get information from models (keypoints, descriptors
 */
bool LocalRecognizer::prepareModels() {

  for( int i = 0; i < mModels.size(); ++i ) {
    prepareModel( mModels[i], 0.01f );
  }
  
  printf("\t * Models prepared \n");
}

/**
 * @function prepareModel
 * @brief Set normal, keypoints and descriptor
 */
void LocalRecognizer::prepareModel( model &_model, float _radius ) {

  pcl::PointCloud<int> indices;
  pcl::UniformSampling<PointType> us;
  
  
  // Compute normals
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud( _model.points );
  norm_est.compute( *_model.normals );
  
  // Extract uniform keypoints
  us.setInputCloud( _model.points );
  us.setRadiusSearch( _radius ); // HERE
  us.compute( indices ); 
  pcl::copyPointCloud( *_model.points, 
		       indices.points, 
		       *_model.keypoints );
  
    // Find descriptors for each keypoint
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> de;
    de.setRadiusSearch( mDescriptor_Radius );

    de.setInputCloud( _model.keypoints );
    de.setInputNormals( _model.normals );
    de.setSearchSurface( _model.points );
    de.compute( *_model.descriptors );
}


/**
 * @function
 * @brief
 */
void LocalRecognizer::setScene( pcl::PointCloud<PointType>::Ptr _scene ) {
  mScene.points = _scene;
  prepareModel( mScene, 0.03f );
}

/**
 * @function
 * @brief
 */
bool LocalRecognizer::matching() {

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud( mModels[0].descriptors );
  
  // For each keypoint in the scene, find nearest neighbor in the model
  // keypoint descriptors and add it to the correspondence vector
  for( size_t i = 0; i < mScene.descriptors->size(); ++i ) {
    std::vector<int> n_inds(1);
    std::vector<float> n_sqr_dists(1);
    if( !pcl_isfinite( mScene.descriptors->at(i).descriptor[0] ) ) { continue; }
    int found_neighbors = match_search.nearestKSearch( mScene.descriptors->at(i), 1,
						       n_inds, n_sqr_dists );
    if( found_neighbors == 1 && n_sqr_dists[0] < 0.25f ) {
      pcl::Correspondence corr( n_inds[0], static_cast<int>(i), n_sqr_dists[0] );
      mModel_scene_corrs->push_back( corr );
    }

  }
  std::cout << "Correspondences found: "<< mModel_scene_corrs->size() << std::endl;
}

/**
 * @function correspondenceGrouping
 * @brief Discard matches that do not make sense geometrically
 */
bool LocalRecognizer::correspondenceGrouping() {

  std::vector<pcl::Correspondences> clustered_corrs;

  pcl::GeometricConsistencyGrouping<PointType,PointType> gc;
  gc.setGCSize( mCg_size );
  gc.setGCThreshold( mCg_thresh );
  
  gc.setInputCloud( mModels[0].keypoints );
  gc.setSceneCloud( mScene.keypoints );
  gc.setModelSceneCorrespondences( mModel_scene_corrs );

  gc.recognize( mRotTrans, clustered_corrs );

  printf("Model instances found: %d \n", mRotTrans.size() );

  return true;
}



/**
 * @function printInfo
 * @brief Print models info
 */
void LocalRecognizer::printInfo() {

  for( int i = 0; i < mModels.size(); ++i ) {
    std::cout << "********************** " << std::endl;
    std::cout << "Model "<< i << std::endl;
    std::cout << "Model name: "<< mModels[i].name << std::endl;
    std::cout << "Model num points: "<< mModels[i].points->points.size() << std::endl;
  }

}


/**
 * @function loadModel
 */
model LocalRecognizer::loadModel( Json::Value var ) {
  
  model mo;

  // Get name
  std::string name = var.get("name", "Empty").asString();
  mo.name = name;

  // Get path
  std::string path = var.get("path", "Empty").asString();
  mo.path = path;
  
  // Load pcd
  pcl::PointCloud<PointType>::Ptr points( new pcl::PointCloud<PointType>() );
  if( pcl::io::loadPCDFile( path, *points ) == -1 ) {
    std::cout <<"Could not load "<<mo.name << std::endl;
  }
  mo.points = points;
  return mo;
}

/**
 * @function viewRecognition
 * @brief View correspondences
 */
void LocalRecognizer::viewRecognition() {
  
  pcl::visualization::PCLVisualizer viewer ("Local Recognizer");
  viewer.addPointCloud ( mScene.points, "scene_cloud");

  for (size_t i = 0; i < mRotTrans.size (); ++i) {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*mModels[0].points, *rotated_model, mRotTrans[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return;
}
