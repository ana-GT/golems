
#include <caffe/caffe.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <memory>

typedef std::pair<std::string, float> Prediction;

/**
 * @class Classifier
 */
class Classifier {

 public:
  Classifier();
  void init( const std::string &_model_file,
	     const std::string &_trained_file,
	     const std::string &_mean_file,
	     const std::string &_label_file );
  
  std::vector<Prediction> classify( const cv::Mat &_img,
                                    int &_idx,
				    int N = 5 );
  std::vector<float> ExtractFeatures( const cv::Mat &_img );  

 private:
  void SetMean( const std::string &_mean_file );
  std::vector<float> Predict( const cv::Mat &_img );
  void WrapInputLayer( std::vector<cv::Mat>* _input_channels );
  void Preprocess( const cv::Mat &_img,
		   std::vector<cv::Mat>* _input_channels );

 private:
  std::shared_ptr< caffe::Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
  std::vector<std::string> labels_;
  bool mInitFlag;
};
