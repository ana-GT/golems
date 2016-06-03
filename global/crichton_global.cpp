
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

/*
// Friday, October 23rd, 2015 - Crichton 
 gTrk[0].linear() <<   0.0248514, 0.833346, 0.552193,   
   0.999686, -0.0188633, -0.0165231,    
-0.00335323, 0.55243, -0.833553;   

 gTrk[0].translation() << 0.162557, 0.051494, 0.130277; 

*/
/*
// Perception_Pick
 gTrk[0].linear() << -0.0149254,   0.782269,   0.622762,   
  0.997846,  0.0514428, -0.0407039,  
 -0.063878,   0.620814,  -0.781351;

 gTrk[0].translation() << 0.164321, 0.0616093, 0.155298;
*/  
/*
  // Perception Pick FTTS (flipt x and y axes signs (put negative to 1 and 2 column on rot mat)
 gTrk[0].linear() <<   -0.0223635,    -0.798772,    0.601218,    
   -0.999673,  0.0253163, -0.00354995,   
   -0.012385,      -0.6011,   -0.799077;

 gTrk[0].translation() << 0.153776, 0.0295767, 0.131247;
*/
// Measured Sunday - April 24th
gTrk[0].linear() << -0.0262939,   -0.793538,   0.607953,   
  -0.999654, 0.0207653, -0.0161307,  
0.000176026,   -0.608167,  -0.793809;   

gTrk[0].translation() <<  0.145756, 0.0476029, 0.127474;
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
  gKnownConfs[RIGHT_MSG][RESTING] << 0, M_PI*30.0/180.0, 0, M_PI*45.0/180.0, 0, 0, -M_PI*90.0/180.0;

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
