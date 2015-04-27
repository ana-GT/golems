
#include "crichton_global.h"

Eigen::Isometry3d gTrk[2];
Eigen::VectorXd gKnownConfs[NUM_SIDES_MSG][TOTAL_CONF];


void loadGlobals_golems() {    

gTrk[0].linear() <<  0.0263018,   0.897194,   0.440852,   
  0.999606, -0.0279231, -0.0028104,  
0.00978852,   0.440753,  -0.897575;    
gTrk[0].translation() <<0.157548, 0.0393149, 0.10766;

/*
// Crichton saturday, April 25th, 2015
gTrk[0].linear() << 0.0261778,  0.894997,  0.445303,  
 0.999258, -0.036016, 0.0136444, 
0.0282497,  0.444615, -0.895276;    
 
gTrk[0].translation() << 0.157097, 0.0303937 , 0.10595;
*/

/*
// ALITA: Wednesday, April 22nd, 2015
gTrk[0].linear() << -0.0091301,   -0.061887,   -0.998041,     
  0.0734777,    0.995343,  -0.0623919,   
   0.997255,  -0.0739034, -0.00454027;    
 
gTrk[0].translation() << 1.28965, 0.0898923, 0.518498;
*/
/*
  gTrk[0].linear() << 0.056131,    0.819654,    0.570102,    
   0.998017,   -0.062359, -0.00860709,   
  0.0284961,    0.569454,   -0.821529;    

 gTrk[0].translation() <<  0.148734, 0.0361318, 0.115795;
*/
/*
 gTrk[0].linear() <<  0.0468941,    0.846075,    0.530997,    
   0.998702,  -0.0502984, -0.00805472,   
  0.0198934,    0.530685,   -0.847336;    
     
 gTrk[0].translation() << 0.152552, 0.0373204, 0.115285;
*/
/*
 gTrk[0].linear() << 0.0373284, 0.290763, 0.956067,   
  0.998929, -0.0370362, -0.0277383,  
 0.0273439, 0.956078, -0.291834;
 gTrk[0].translation() << 0.109844, 0.0232314, 0.152649;
*/
/*
 gTrk[0].linear() <<  0.0459043,   0.889303,   0.455009,    
  0.998875, -0.0462777, -0.0103245,  
  0.0118752,   0.454972,  -0.890427; 

   gTrk[0].translation() << 0.15974, 0.0368649, 0.113548;  

   gTrk[1].linear() <<   -0.0404581, -0.885179,  -0.463488,
   -0.998769, 0.0491498, -0.00668417,
   0.028697,  0.462647,  -0.886078;   
   gTrk[1].translation() << -0.154896, -0.000794479, 0.136019;
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
