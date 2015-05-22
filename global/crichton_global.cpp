
#include "crichton_global.h"

Eigen::Isometry3d gTrk[2];
Eigen::VectorXd gKnownConfs[NUM_SIDES_MSG][TOTAL_CONF];

void loadGlobals_golems() {
/*
gTrk[0].linear() <<  0.0346732,   0.689375,   0.723574,   
  0.999182, -0.0390024, -0.0107211,  
 0.0208303,   0.723354,  -0.690163;    
gTrk[0].translation() <<0.140992, 0.0357854, 0.13118;
*/


  gTrk[0].linear() <<   0.996494,  0.0371087,  0.0749833,  
 -0.014566,   0.959515,  -0.281282,  
-0.0823855,   0.279204,   0.956691;  

  gTrk[0].translation() << -0.145421, 0.0612711, 0.0821505;

/*
  gTrk[0].linear() <<     0.933979,   0.127202,    0.33392,  
-0.0044235,   0.938537,   -0.34515,   
   -0.3573,   0.320886,   0.877136;   

  gTrk[0].translation() <<  -0.254362, 0.154248, 0.175327;
*/
/*
  gTrk[0].linear() <<   0.0406064,    0.824616,    0.564234,    
    0.99893,  -0.0460236, -0.00462788,   
    0.0221518,    0.563818,   -0.825602;      
  
  gTrk[0].translation() << 0.144254, 0.0331198, 0.115924;
*/

/*
gTrk[0].linear() <<  0.987236, -0.0643039,  -0.145705,  
  0.117983,   0.909823,   0.397871,   
  0.106981,  -0.409984,   0.905797;   

gTrk[0].translation() << 0.0734403, 0.111031, 0.092753;
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
