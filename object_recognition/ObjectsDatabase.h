/**
 * @file ObjectsDatabase.h
 */
#pragma once

#include <string>
#include <map>
#include "perception/pointcloud_tools/sq_fitting/SQ_parameters.h"
#include <object_recognition/base_classifier.h>


struct ObjectEntry{
  std::string name;
  int sq_type; // REGULAR, TAMPERED, BENT, MULTIPLE
  // In case it is multiple
  int num_parts;
  std::vector<int> part_type;
  int hint_search; // perpendicular-to-Z , containing-Z
};


/**
 * @class ObjectsDatabase
 */
class ObjectsDatabase {

 public:
 
  void init_classifier();
  void load_explanations();
  void load_dataset();
  std::vector<Prediction> classify( cv::Mat _img,
				    int &_index,
				    std::string &_label );
  int getSQtype( int _index ) { 
    return mDataset[mNames[_index]].sq_type;
  }
  ObjectEntry getEntry( int _index ) { return mDataset[mNames[_index]]; }
  void sayIt( int _index );
  
  void setModelFile( char* _model_file ) { mModel_file = std::string(_model_file); }

 protected:
  Classifier mClassifier;
  std::string mModel_file;
  std::string mTrain_file;
  std::string mMean_file;
  std::string mLabel_file;
  std::string mExplanations;
  std::vector<std::string> mHabla;  
  std::vector<std::string> mNames;

  int mNUM_OBJECTS;
  std::map<std::string, ObjectEntry> mDataset;


};

