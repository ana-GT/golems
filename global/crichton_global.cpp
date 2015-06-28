
#include "crichton_global.h"

Eigen::Isometry3d gTrk[2];
Eigen::VectorXd gKnownConfs[NUM_SIDES_MSG][TOTAL_CONF];

void loadGlobals_golems() {   


// Alita
/*
 gTrk[0].linear() <<  0.988989, 0.00331416,  -0.147952, 
-0.0680002,   0.898135,   -0.43443,   
  0.131441,  0.439707,   0.888471;  

 gTrk[0].translation() << -0.0203929, 0.049209, 0.0567504;
*/
/*
gTrk[0].linear() << 0.974359, -0.0227323,  -0.223849,  
-0.0811584, 0.892399, -0.443888,  
  0.209853, 0.450673, 0.867672;

  gTrk[0].translation() << 0.0216599, 0.0576096, 0.0798085;
*/
// Thursday, June 18th, 2015 - Crichton 
 gTrk[0].linear() <<  0.0396399,   0.878365,   0.476344,   
  0.998917, -0.0464489, 0.00252368,  
 0.0243424,   0.475729,  -0.879255;   

 gTrk[0].translation() << 0.172434, 0.0242993, 0.125959;

// Crichton: Monday June 22
gTrk[1].linear() << -0.0406066,  -0.882481,  -0.468593,  
 -0.997766,  0.0607085, -0.0278667,  
 0.0530394,   0.466415,  -0.882975;   

gTrk[1].translation() << -0.176772, 0.0166431, 0.142114;


// Wednesday, June 10th, 2015
/*
 gTrk[0].linear() << 0.0312785,   0.877215,   0.479077,   
  0.998764, -0.0459603,  0.0189474,  
 0.0386394,   0.477892,  -0.877568;   

 gTrk[0].translation() << 0.176055, 0.0164966, 0.120763;
*/
/*
 0.0172832    0.88074   0.473285   0.178149
  0.999211 -0.0321405  0.0233217 0.00966935
  0.035752   0.472508  -0.880601   0.124434
         0          0          0          1
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
