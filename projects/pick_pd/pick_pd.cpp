/**
 * @file pick_pd.cpp
 */
#include "pick_pd.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>

std::string gBinNames[NUM_BINS] = { "bin_A", "bin_B", "bin_C", "bin_D", "bin_E",
				    "bin_F", "bin_G", "bin_H", "bin_I", "bin_J",
				    "bin_K", "bin_L" };

std::string gObjNames[NUM_OBJECT_TYPES] = { "oreo_mega_stuf",
					    "champion_copper_plus_spark_plug",
					    "expo_dry_erase_board_eraser",
					    "kong_duck_dog_toy",
					    "genuine_joe_plastic_stir_sticks",
					    "munchkin_white_hot_duck_bath_toy",
					    "crayola_64_ct",
					    "mommys_helper_outlet_plugs",
					    "sharpie_accent_tank_style_highlighters",
					    "kong_air_dog_squeakair_tennis_ball",
					    "stanley_66_052",
					    "safety_works_safety_glasses",
					    "dr_browns_bottle_brush",
					    "laugh_out_loud_joke_book",
					    "cheezit_big_original",
					    "paper_mate_12_count_mirado_black_warrior",
					    "feline_greenies_dental_treats",
					    "elmers_washable_no_run_school_glue",
					    "mead_index_cards",
					    "rolodex_jumbo_pencil_cup",
					    "first_years_take_and_toss_straw_cup",
					    "highland_6539_self_stick_notes",
					    "mark_twain_huckleberry_finn",
					    "kyjen_squeakin_eggs_plush_puppies",
					    "kong_sitting_frog_dog_toy" };
  

/**
 * @function ~Pick_PD
 * @brief Constructor
 */
Pick_PD::Pick_PD() {

  for( int i = 0; i < NUM_BINS; ++i ) {
    mBinNames[ gBinNames[i] ] = i;
  } 

  for( int i = 0; i < NUM_OBJECT_TYPES; ++i ) {
    mObjTypes[ gObjNames[i] ] = i;
  } 

  
}

/**
 * @function ~Pick_PD
 * @brief Destructor
 */
Pick_PD::~Pick_PD() {}

/**
 * @function read_taskfile
 * @brief Read input file
 */
bool Pick_PD::read_taskfile( const std::string &_filename ) {

  Json::Value root;
  Json::Reader reader;
  
  std::ifstream obj_string( _filename.c_str(),
			    std::ifstream::binary );
  
  bool parsingSuccessful = reader.parse( obj_string, root );
  if( !parsingSuccessful ) {
    std::cout << "OH CRAP! Failed to parse file: \n "<< reader.getFormattedErrorMessages()<<std::endl;
    return false;
  }

  //-- Read bin contents
  Json::Value bin_contents = root["bin_contents"];

  std::map<std::string, int>::iterator it;
  for( it = mBinNames.begin(); it != mBinNames.end(); ++it ) {
    
    const Json::Value bin = bin_contents[ it->first ];
    std::vector<int> content;
    
    for( int j = 0; j < bin.size(); ++j ) {
      std::string name = bin[j].asString();
      content.push_back( mObjTypes[name] );
    }

    mBinContent[it->second] = content;
  }

  //-- Debug bin contents
  print_bin_contents();

  //-- Read task
  Json::Value work_order = root["work_order"];

  for( int i = 0; i < work_order.size(); ++i ) {
    Json::Value task = work_order[i];
    
    std::string bin = task["bin"].asString();
    std::string item = task["item"].asString();

    // Store
    std::pair<int,int> t;
    t.first = mBinNames[bin];
    t.second = mObjTypes[item];
    mTask.push_back( t );
  }

  //-- Debug work order
  print_task();
  
}


/**
 * @function taskIsDefined
 * @brief If the task vector has at least a pair [bin,object]
 */
bool Pick_PD::taskIsDefined() {
  if( mTask.size() > 1 ) { return true; }

  return false;
}

/**
 * @function print_bin_contents
 * @brief Print the contents of each bin in order
 */
void Pick_PD::print_bin_contents() {

  for( int i = 0; i < NUM_BINS; ++i ) {
    std::cout << " "<<gBinNames[i]<<": \n";
    for( int j = 0; j < mBinContent[i].size(); ++j ) {
      std::cout << "\t " << gObjNames[mBinContent[i][j]] << std::endl;
    } 
    std::cout << std::endl;
  }

  
}

/**
 * @function print_task
 * @brief Print the task defined as a series of bin + object to pick (in execution order)
 */
void Pick_PD::print_task() {

  for( int i = 0; i < mTask.size(); ++i ) {
    std::cout << gBinNames[mTask[i].first] << " : "
	      << gObjNames[mTask[i].second] << std::endl;
  }  
}

/**
 * @function setTransformations
 */
void Pick_PD::setTransformations() {

  // Get Tf from leftmost bins
  mToi[0].setIdentity();
  mToi[0].matrix() << 0,0,-1, 0.44,
    -1,0,0, -0.27,
    0,1,0, (0.78+0.99-0.135),
    0,0,0,1;

  // Vertical
  Eigen::Isometry3d Tv;
  Tv.setIdentity(); Tv.translation() << 0, -0.25, 0;
  
  mToi[3] = mToi[0]*Tv;
  mToi[6] = mToi[3]*Tv;
  mToi[9] = mToi[6]*Tv;
  
  // Horizontal
  Eigen::Isometry3d Th;
  Th.setIdentity(); Th.translation() << -0.27, 0, 0;

  for( int i = 0; i <= 9 ; i = i+3 ) {
    mToi[i+1] = mToi[i]*Th;
    mToi[i+2] = mToi[i+1]*Th;
  }
  
}
