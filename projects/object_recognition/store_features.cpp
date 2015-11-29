#include <caffe/caffe.hpp>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "classifier.h"
#include <fstream>

cv::Mat gRgbImg;
std::string gWindowName("store_features");
char* gModelRoot("/home/ana/Software/caffe/models");
char gModel_file[200];
char gTrain_file[200];
char gMean_file[200];
char gLabel_file[200];
char gTraining_images[200];
char gFeature_file[200];

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  // http://www.robots.ox.ac.uk/~vgg/research/very_deep/
  sprintf( gModel_file, "%s/VGG_ILSVRC_19_layers/VGG_ILSVRC_19_layers_deploy.prototxt",
	   gModelRoot );
  sprintf( gTrain_file, "%s/VGG_ILSVRC_19_layers/VGG_ILSVRC_19_layers.caffemodel", gModelRoot );
  sprintf( gMean_file, "%s/../data/ilsvrc12/imagenet_mean.binaryproto", gModelRoot );
  sprintf( gLabel_file, "%s/../data/ilsvrc12/synset_words.txt", gModelRoot );
  sprintf( gTraining_images, "/home/ana/Research/golems/YCB_data/training_images.txt" );
  sprintf( gFeature_file, "/home/ana/Research/golems/YCB_data/feature_files.yml" );

  Classifier gClassifier( gModel_file, gTrain_file, gMean_file, gLabel_file );
    
  cv::namedWindow( gWindowName, cv::WINDOW_AUTOSIZE );
 
  ::google::InitGoogleLogging( argv[0] );

  // Read and store features
  std::ifstream input;
  input.open( gTraining_images, std::ifstream::in ); 
  
  cv::FileStorage fs(gFeature_file, cv::FileStorage::WRITE );

  int count = 0;
  if( input.is_open() ) {

    fs << "features" << "[";
    while( !input.eof() ) {
      std::string file, label;
      input >> file >> label;
      if( !input.eof() ) {

	// Read image
	gRgbImg = cv::imread( file, -1 );
	// Show it for debugging
	cv::imshow( gWindowName, gRgbImg );
	cv::waitKey(30);
	// Get features
	std::vector<float>features;
	features = gClassifier.ExtractFeatures( gRgbImg );
	// Save
	fs << "{:"<<"filename" << file<<"label"<<label<<"feature"<<"[:";
	printf("Feature size: %d \n", features.size() );
	for( int i=0; i < features.size(); ++i ) {
	  fs << features[i];
	}
	fs << "]" <<"}";
	// Let me know how we are doing
	printf("[%d] Saving feature of item %s (%s) \n", count, 
	       label.c_str(),
	       file.c_str() );
	count++;
      } // if read image is valid (did not hit EOF yet)
    } // end while

    fs << "]";
    fs.release();

  } else {
    printf("Input was never opened! \n");
  }
 
  printf("Finished getting the features! \n");

}
