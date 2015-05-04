/**
 * @file pick_pd.h
 */
#pragma once

#include <vector>
#include <utility>
#include <string>
#include <map>
#include <Eigen/Geometry>

#define VAR_NAME_TO_STRING(var) #var
#define NUM_BINS 12
#define NUM_OBJECT_TYPES 25

extern std::string gBinNames[NUM_BINS];
extern std::string gObjNames[NUM_OBJECT_TYPES];

/**
 * @class Pick_PD
 */
class Pick_PD {

 public:
  Pick_PD();
  ~Pick_PD();

  bool read_taskfile( const std::string &_filename );
  bool taskIsDefined();

  void print_bin_contents();
  void print_task();

  // Pod transformations
  void set_Two( const Eigen::Isometry3d &_Two ) { mTwo = _Two; this->setTransformations(); }
  void setTransformations();
  Eigen::Isometry3d getTwi( int _i ) { return mTwo*mToi[_i]; }
  
 private:

  // Pod transformations
  Eigen::Isometry3d mTwo;
  Eigen::Isometry3d mToi[NUM_BINS];

  
  
  // Input data
  std::vector<int> mBinContent[NUM_BINS];
  std::vector<std::pair<int,int> > mTask;

  std::map<std::string,int> mBinNames;
  std::map<std::string,int> mObjTypes;
  
};
