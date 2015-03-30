
#include "crichton_global.h"

Eigen::Isometry3d gTrk[2];

void loadGlobals_golems() {    

 gTrk[0].linear() <<  0.0468941,    0.846075,    0.530997,    
   0.998702,  -0.0502984, -0.00805472,   
  0.0198934,    0.530685,   -0.847336;    
     
 gTrk[0].translation() << 0.152552, 0.0373204, 0.115285;

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
}
