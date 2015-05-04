/**
 * @file test2_fsa.h
 */
#pragma once

#include <golems/global/fsa_data.h>

/**
 * @class Test2_FSA
 */
class Test2_FSA {

 public:
  Test2_FSA();
  ~Test2_FSA();
  void loadRules();
  
  int evaluate( int _msg_source, int _msg );
  void resetFSA() { mState = LISTENING; }
  std::string getCurrentStateName();
  
 private:
  
  int mState;
  int mRules[TOTAL_STATES][4];  
  
};
