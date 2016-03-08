/**
 * @file ObjectsDatabase.cpp
 */

#include "ObjectsDatabase.h"


/**
 * @function fill_dataset
 */
void ObjectsDatabase::load_dataset() {

  mNUM_OBJECTS = 42;
  
  std::string tNames[] = { "apple", "banana", "brine_ball","campbell_soup", "cheezit", 
			   "coffee_mate", "coke_can", "domino", "drill", "elmer_glue", 
			   "fluorescent_ball", "hammer", "jar", "jello_chocolate", 
			   "jello_strawberry", "juice_box", "lemon", "light_blue_brush", 
			   "magic_cube", "master_chef_coffee", "mike_monster", "milk", 
			   "mustard", "orange", "peach", "pear", "pink_bate", 
			   "plum", "pringles", "red_bowl", "red_cup", "red_plate", 
			   "soft_scrub", "spam", "strawberry", "sun_maid_raisins", 
			   "tennis_ball", "white_cup", "wooden_cube", "wood_glue", 
			   "yellow_cone", "yellow_cuppie" };
  mNames.insert( mNames.end(), tNames, tNames + mNUM_OBJECTS );

  int tSQ_types[] = {REGULAR,BENT,REGULAR,REGULAR,REGULAR,
		     REGULAR,REGULAR,REGULAR,MULTIPLE,REGULAR,
		     REGULAR,MULTIPLE,REGULAR,REGULAR,
		     REGULAR, REGULAR,REGULAR,MULTIPLE,
		     REGULAR, REGULAR,REGULAR,REGULAR,
		     REGULAR,REGULAR,REGULAR,TAMPERED,TAMPERED,
		     REGULAR,REGULAR,REGULAR,REGULAR,REGULAR,
		     REGULAR,REGULAR,REGULAR,REGULAR,
		     REGULAR,TAMPERED,REGULAR,REGULAR,
		     TAMPERED,REGULAR};
  
  if( mDataset.size() == mNUM_OBJECTS ) {
    printf("mDataset already full \n");
    return;
  }
  
  for( int i = 0; i < mNUM_OBJECTS; ++i ) {

    ObjectEntry oe;
    oe.name = mNames[i];
    oe.sq_type = tSQ_types[i];
    mDataset[mNames[i]] = oe;
  }

}



/**
 * @function ObjectsDatabase
 */
void ObjectsDatabase::init_classifier() {
  mModel_file = std::string("/home/ana/Desktop/Crichton_data_trained/deploy_alexnet.prototxt");
  mTrain_file = std::string("/home/ana/Desktop/Crichton_data_trained/partial_alexnet_iter_2000.caffemodel");
  mMean_file = std::string("/home/ana/Desktop/Crichton_data_processed/Crichton_data_227_brute_resize_train.binaryproto");
  mLabel_file = std::string("/home/ana/Desktop/Crichton_data/training_labels.txt");
  mExplanations = std::string("/home/ana/Desktop/Crichton_data/training_labels_explanatory.txt");

  // Remember AlexNet, referenc_caffenet and RCNN_ilsvrc13: 227, googlenet: 224
  // REMEMBER ABOVE WHEN LOADING MEAN FILE
  mClassifier.init( mModel_file, mTrain_file, mMean_file, mLabel_file );
 
  load_explanations();
}

/**
 * @function load_explanations
 * @brief Load verbal expressions for the robot to utter (optionally) per each recognized object
 */
void ObjectsDatabase::load_explanations() {

  std::ifstream  explan( mExplanations, std::ifstream::in );

  std::string line;
  std::string words;
  while( std::getline(explan, line) ) {
    std::size_t pos = line.find("Veo");
    words = line.substr(pos);
    mHabla.push_back(words);
  }
  
  explan.close();
  printf("Loaded explanations. Size: %lu \n", mHabla.size());

}

/**
 * @function classify
 * @brief Return the most likely object detected
 */
std::vector<Prediction> ObjectsDatabase::classify( cv::Mat _img,
						   int &_index,
						   std::string &_label ) {

  std::vector<Prediction> predictions = mClassifier.classify( _img, _index );
  _label = predictions[0].first;	
  return predictions;
}

/**
 * @function sayIt
 * @brief Speech saying the object found
 */
void ObjectsDatabase::sayIt( int _index ) {
  char cmd[150];
  sprintf(cmd, "espeak '%s' -p 80 -s 200 -ves-la ", mHabla[_index].c_str() );
  system(cmd); usleep(0.1*1e6);
}
