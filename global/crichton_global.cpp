
#include "crichton_global.h"

Eigen::Isometry3d gTrk[2];
Eigen::VectorXd gKnownConfs[NUM_SIDES_MSG][TOTAL_CONF];

void loadGlobals_golems() {   

 gTrk[0].linear() <<    0.998801,  -0.0481417, -0.00887832,  
  0.0231378,    0.624079,   -0.781019,    
  0.0431403,    0.779877,    0.624445;    

gTrk[0].translation() << -0.0725424, 0.135835, 0.139225;
/*
  gTrk[0].linear() <<   0.0348735,   0.895301,   0.444095,   
  0.999206,  -0.039791, 0.00175427,  
 0.0192416,   0.443682,  -0.895978;

gTrk[0].translation() << 0.152917, 0.0402138, 0.105963;
*/
 
 //   ARC_UP = 0
 gKnownConfs[LEFT_MSG][ARC_UP].resize(7);
 gKnownConfs[LEFT_MSG][ARC_UP] << 90*M_PI/180.0, -30*M_PI/180.0, 0, -60*M_PI/180.0, 0, -40*M_PI/180.0, 0;
 gKnownConfs[RIGHT_MSG][ARC_UP].resize(7);
 gKnownConfs[RIGHT_MSG][ARC_UP] << -90*M_PI/180.0, 30*M_PI/180.0, 0, 60*M_PI/180.0, 0, 40*M_PI/180.0, 0;

 //   FRONTAL_FIST = 1
  gKnownConfs[LEFT_MSG][FRONTAL_FIST].resize(7);
  gKnownConfs[LEFT_MSG][FRONTAL_FIST] << 0, M_PI*-50.0/180.0, 0, M_PI*-75.0/180.0, 0, 0, M_PI*60.0/180.0;
  gKnownConfs[RIGHT_MSG][FRONTAL_FIST].resize(7);
  gKnownConfs[RIGHT_MSG][FRONTAL_FIST] << 0, M_PI*50.0/180.0, 0, M_PI*75.0/180.0, 0, 0, -M_PI*120.0/180.0;

  //  RESTING = 2
  gKnownConfs[LEFT_MSG][RESTING].resize(7);
  gKnownConfs[LEFT_MSG][RESTING] << 0, M_PI*-30.0/180.0, 0, M_PI*-45.0/180.0, 0, 0, M_PI*30.0/180.0;  
  gKnownConfs[RIGHT_MSG][RESTING].resize(7);
  gKnownConfs[RIGHT_MSG][RESTING] << 0, M_PI*30.0/180.0, 0, M_PI*45.0/180.0, 0, 0, -M_PI*90.0/180.0;;

  // LOOKUP_SKY = 3,
  gKnownConfs[LEFT_MSG][LOOKUP_SKY].resize(7);
  gKnownConfs[LEFT_MSG][LOOKUP_SKY] << M_PI*90.0/180.0, -M_PI*90.0/180.0, 0, 0, 0, 0, 0;
  gKnownConfs[RIGHT_MSG][LOOKUP_SKY].resize(7);
  gKnownConfs[RIGHT_MSG][LOOKUP_SKY] << -M_PI*90.0/180.0, M_PI*90.0/180.0, 0, 0, 0, 0, 0; 

  // DEMANDING = 4
  gKnownConfs[LEFT_MSG][DEMANDING].resize(7);
  gKnownConfs[LEFT_MSG][DEMANDING] << 10*M_PI/180.0, -85*M_PI/180.0, 0*M_PI/180.0, -45*M_PI/180.0, 0, -30*M_PI/180.0, -50*M_PI/180.0;  
  gKnownConfs[RIGHT_MSG][DEMANDING].resize(7);
  gKnownConfs[RIGHT_MSG][DEMANDING] <<  -16*M_PI/180.0, 67*M_PI/180.0, -40*M_PI/180.0, 50*M_PI/180.0, 0, 40*M_PI/180.0, 50*M_PI/180.0;

    // HUG = 5
  gKnownConfs[LEFT_MSG][HUG].resize(7);
  gKnownConfs[LEFT_MSG][HUG] << 0, -60*M_PI/180.0, 0, -60*M_PI/180.0, 0, 30*M_PI/180.0, 0;  
  gKnownConfs[RIGHT_MSG][HUG].resize(7);
  gKnownConfs[RIGHT_MSG][HUG] << 0, 60*M_PI/180.0, 0, 60*M_PI/180.0, 0, -30*M_PI/180.0, 0;

 
}
