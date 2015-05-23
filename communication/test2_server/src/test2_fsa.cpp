/**
 * @file test2_fsa.cpp 
 */
#include <stdio.h>
#include <string>
#include "test2_fsa.h"

Test2_FSA::Test2_FSA() {
  mState = SLEEPING;
  loadRules();
}

Test2_FSA::~Test2_FSA() {

}

/**
 * @function loadRules
 */
void Test2_FSA::loadRules() {
  // 0: From whom I receive changer msg 1: What type of message this is 2: My new status 3: Should I do something when I get here?
  mRules[SLEEPING][0] = CLIENT; mRules[SLEEPING][1] = CONNECTED_MSG; mRules[SLEEPING][2] = LISTENING; mRules[SLEEPING][3] = NO_ACTION;
  mRules[LISTENING][0] = CLIENT; mRules[LISTENING][1] = START_MSG; mRules[LISTENING][2] = START_TASK; mRules[LISTENING][3] = NO_ACTION;
  mRules[START_TASK][0] = SEE_MODULE; mRules[START_TASK][1] = RCV_IMAGE_MSG; mRules[START_TASK][2] = WAIT_INPUT_CLIENT; mRules[START_TASK][3] = SEND_IMAGE_REQUEST;
  mRules[WAIT_INPUT_CLIENT][0] = CLIENT; mRules[WAIT_INPUT_CLIENT][1] = RCV_INPUT_MSG; mRules[WAIT_INPUT_CLIENT][2] = GOT_INPUT_CLIENT; mRules[WAIT_INPUT_CLIENT][3] = SEND_INPUT_REQUEST;  
  mRules[GOT_INPUT_CLIENT][0] = PLAN_MODULE; mRules[GOT_INPUT_CLIENT][1] = PLAN_PREP_DONE_MSG; mRules[GOT_INPUT_CLIENT][2] = PLAN_PREP_READY; mRules[GOT_INPUT_CLIENT][3] = SEND_PREP_REQUEST_PLAN;
  mRules[PLAN_PREP_READY][0] = SEE_MODULE; mRules[PLAN_PREP_READY][1] = SENT_OBJ_MSG; mRules[PLAN_PREP_READY][2] = PLAN_READY; mRules[PLAN_PREP_READY][3] = SEND_OBJ_COMMAND;
  mRules[PLAN_READY][0] = PLAN_MODULE; mRules[PLAN_READY][1] = PLAN_DONE_MSG; mRules[PLAN_READY][2] = PLAN_DONE; mRules[PLAN_READY][3] = START_PLANNING_COMMAND;
  mRules[PLAN_DONE][0] = CLIENT; mRules[PLAN_DONE][1] = GO_BACK_MSG; mRules[PLAN_DONE][2] = LISTENING; mRules[PLAN_DONE][3] = NO_ACTION;

}

/**
 * @function getCurrentStateName
 */
std::string Test2_FSA::getCurrentStateName() {

  switch( mState ) {

  case SLEEPING: return std::string("SLEEPING"); break;
  case LISTENING: return std::string("LISTENING"); break;
  case START_TASK: return std::string("START_TASK"); break;
  case WAIT_INPUT_CLIENT: return std::string("WAIT_INPUT_CLIENT"); break;
  case GOT_INPUT_CLIENT: return std::string("GOT_INPUT_CLIENT"); break;
  case PLAN_PREP_READY: return std::string("PLAN_PREP_READY"); break;
  case PLAN_READY: return std::string("PLAN_READY"); break;
  case PLAN_DONE: return std::string("PLAN_DONE"); break;
  }
  
}

/**
 * @function evaluate
 */
int Test2_FSA::evaluate( int _msg_source,
			  int _msg ) {
  printf("Evaluating: Expecting msg data: %d and %d. Got %d and %d \n",
	 mRules[mState][0], mRules[mState][1],
	 _msg_source, _msg );
  if( mRules[mState][0] == _msg_source &&
      mRules[mState][1] == _msg ) {
    printf("\t Transiting from state %d to %d \n", mState, mRules[mState][2]);
    mState = mRules[mState][2];
    return mRules[mState][3];
  }

  return NO_ACTION;
}
