/**
 * @file LocalRecognizer.h
 * @brief Object recognition class using global descriptors
 */

#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_omp.h>

#include "jsoncpp/json/json.h"
#include <string>
#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;

struct model {
  
  model();
  
  std::string name;
  std::string path;
  pcl::PointCloud<PointType>::Ptr points;
  pcl::PointCloud<PointType>::Ptr keypoints;
  pcl::PointCloud<NormalType>::Ptr normals;
  pcl::PointCloud<DescriptorType>::Ptr descriptors;

};

/**
 * @class LocalRecognizer
 */
class LocalRecognizer {
  
 public:
  LocalRecognizer();
  ~LocalRecognizer();
  bool load( std::string _obj_input );
  model loadModel( Json::Value var );
  bool prepareModels();
  void prepareModel( model &_model, float _radius );
  void printInfo();
  void viewModels();
  void viewRecognition();

  void setScene( pcl::PointCloud<PointType>::Ptr _scene );
  bool matching();
  bool correspondenceGrouping();


 private:
  std::vector<model, Eigen::aligned_allocator<model> > mModels;
  model mScene;
  float mRadius; 
  float mDescriptor_Radius;
  float mCg_size;
  float mCg_thresh;
  pcl::CorrespondencesPtr mModel_scene_corrs;  
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > mRotTrans;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
