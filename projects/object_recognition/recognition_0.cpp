#include <caffe/caffe.hpp>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "object_recognition/base_classifier.h"

cv::VideoCapture gCapture;
cv::Mat gRgbImg;
std::string gWindowName("recognition_0");
char* gModelRoot = "/home/ana/Software/caffe/models";
char* gModel_file ="/home/ana/Software/caffe/models/VGG_ILSVRC_19_layers/VGG_ILSVRC_19_layers_deploy.prototxt";
char* gTrain_file = "/home/ana/Software/caffe/models/VGG_ILSVRC_19_layers/VGG_ILSVRC_19_layers.caffemodel";
char* gMean_file = "/home/ana/Software/caffe/data/ilsvrc12/imagenet_mean.binaryproto";
char* gLabel_file = "/home/ana/Software/caffe/data/ilsvrc12/synset_words.txt";

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  // http://www.robots.ox.ac.uk/~vgg/research/very_deep/  
  int c;
  while( (c=getopt(argc,argv,"n:t:m:l:h")) != -1 ) {
    switch(c) {
    case 'n': { gModel_file = optarg; } break;
    case 't': { gTrain_file = optarg; } break;
    case 'm': { gMean_file = optarg; } break;
    case 'l': { gLabel_file = optarg; } break;
    case 'h': { printf("Syntax: %s -n deploy.txt -t trained.caffemodel -m mean.binaryproto -l labels.txt \n",
		       argv[0] ); return 1; } break;
    }
  }

  Classifier gClassifier( gModel_file, gTrain_file, gMean_file, gLabel_file );
  


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
      int idx;
      std::vector<Prediction> predictions = gClassifier.classify( gRgbImg, idx );
      printf("Predictions: \n");
      for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        std::cout << std::fixed << std::setprecision(4) << p.second << " - \""
                  << p.first << "\"" << std::endl;
     
      }


    
    }


  }

}
