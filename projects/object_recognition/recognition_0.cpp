#include <caffe/caffe.hpp>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "object_recognition/ObjectsDatabase.h"

cv::VideoCapture gCapture;
cv::Mat gRgbImg;
std::string gWindowName("recognition_0");

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  ObjectsDatabase mOd;
  mOd.init_classifier();  

  gCapture.open( cv::CAP_OPENNI2 );
  
  if( !gCapture.isOpened() ) {
    printf("\t [ERROR] Could not open the capture object \n");
    return -1;
  }

  gCapture.set( cv::CAP_PROP_OPENNI2_MIRROR, 0.0 );
  gCapture.set( cv::CAP_PROP_OPENNI_REGISTRATION, -1.0 );

  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
 
  ::google::InitGoogleLogging( argv[0] );
  
  for(;;) {

    if( !gCapture.grab() ) {
      printf("\t * ERROR Could not grab a frame \n");
      return -1;
    }

    gCapture.retrieve( gRgbImg, cv::CAP_OPENNI_BGR_IMAGE );
    cv::imshow( gWindowName, gRgbImg );

    char k = cv::waitKey(30);
    if( k == 'q' ) {
      printf("\t Finishing program \n");
      break;
    } else if( k == 'i' ) {
      
      // Predict 
      std::vector<Prediction> predictions; int idx; std::string label;
      predictions = mOd.classify( gRgbImg, idx, label ); 
      printf("Predictions: \n");
      for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        std::cout << std::fixed << std::setprecision(4) << p.second << " - \""
                  << p.first << "\"" << std::endl;
    
      }


    
    }


  }

}
