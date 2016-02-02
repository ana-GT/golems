
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "classifier.h"

/**
 * @function Classifier
 * @brief Constructor
 */
Classifier::Classifier( const std::string &_model_file,
			const std::string &_trained_file,
			const std::string &_mean_file,
			const std::string &_label_file ) {
  
  // In case you compiled with CPU, which I did
  caffe::Caffe::set_mode( caffe::Caffe::CPU );
  
  // Load the network
  net_.reset( new caffe::Net<float>( _model_file, caffe::TEST ) );

  net_->CopyTrainedLayersFrom( _trained_file );
  
  CHECK_EQ( net_->num_inputs(), 1 ) << "Network should have exactly one input.";
  CHECK_EQ( net_->num_outputs(), 1 ) << "Network should have exactly one output.";
  
  printf("Number of input blobs: %d \n", net_->input_blobs().size() );

  caffe::Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK( num_channels_ == 3 || num_channels_ == 1 )  << "Input layer should have 3 (RGB) or 1 (Gray) channels.";
  
  input_geometry_ = cv::Size( input_layer->width(), 
			      input_layer->height() );
  
  /** Load the binaryproto mean file */
  this->SetMean( _mean_file );
  
  /** Load labels */
  std::ifstream  labels( _label_file.c_str() );
  CHECK( labels ) << "Unable to open labels file " << _label_file;
  std::string line;
  
  while( std::getline(labels, line) ) {
    labels_.push_back( std::string(line) );
  }

  caffe::Blob<float>* output_layer = net_->output_blobs()[0];
  printf("Number of output layer blobs: %d \n", net_->output_blobs().size() );
  CHECK_EQ( labels_.size(), output_layer->channels() ) << "Number of labels is different from the output layer dimensions.";

  std::vector<std::string> ln = net_->layer_names();
  for( int i = 0; i < ln.size(); ++i ) {
    std::cout << "Layer "<<i<<": "<< ln[i] << std::endl;
  }

  std::vector<std::string> bn = net_->blob_names();
  for( int i = 0; i < bn.size(); ++i ) {
    std::cout << "Blob "<<i<<": "<< bn[i] << std::endl;
  }

}

static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
  return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
  std::vector<std::pair<float, int> > pairs;
  for (size_t i = 0; i < v.size(); ++i)
    pairs.push_back(std::make_pair(v[i], i));
  std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

  std::vector<int> result;
  for (int i = 0; i < N; ++i)
    result.push_back(pairs[i].second);
  return result;
}



/**
 *
 */
std::vector<Prediction> Classifier::classify( const cv::Mat &_img,
					      int N ) {

  std::vector<float> output = this->Predict( _img );
  N = std::min<int>( labels_.size(), N );
  std::vector<int> maxN = Argmax( output, N );

  std::vector<Prediction> predictions;
  for( int i = 0; i < N; ++i ) {
    int idx = maxN[i];
    predictions.push_back( std::make_pair( labels_[idx],
					   output[idx] ) );
  }
  
  return predictions;

}

//////////////////////////////////////////////

/**
 * @Load the mean file in binaryproto format
 */  
void Classifier::SetMean( const std::string &_mean_file ) {

  caffe::BlobProto blob_proto;
  ReadProtoFromBinaryFileOrDie( _mean_file.c_str(), 
				&blob_proto );
  
  // Convert from blob proto to Blob<float>
  caffe::Blob<float> mean_blob;
  mean_blob.FromProto( blob_proto );
  CHECK_EQ( mean_blob.channels(), num_channels_ ) << "Number of channels of mean file doesn't match input layer";
  
  // The format of the mean file is planar 32-bit float BGR or gray
  std::vector<cv::Mat> channels;
  float* data = mean_blob.mutable_cpu_data();
  
  for( int i = 0; i < num_channels_; ++i ) {
    cv::Mat channel( mean_blob.height(), mean_blob.width(),
		     CV_32FC1, data );
    channels.push_back( channel );
    data += mean_blob.height()*mean_blob.width();
  }

  // Merge the separate channels into a single image
  cv::Mat mean;
  cv::merge( channels, mean );

  // Compute the global mean pixel value and create a mean image
  // filled with this value
  cv::Scalar channel_mean = cv::mean( mean );
  mean_ = cv::Mat( input_geometry_,
		   mean.type(), 
		   channel_mean );
  

}

/**
 * @function ExtractFeatures
 * @brief Extract fc7 features (float vector of length 4096)
 */
std::vector<float> Classifier::ExtractFeatures( const cv::Mat &_img ) {

  caffe::Blob<float>* input_layer = net_->input_blobs()[0];

  input_layer->Reshape( 1, num_channels_,
			input_geometry_.height, 
			input_geometry_.width );

  /* Forward dimension change to all layers. */
  net_->Reshape();
  
  std::vector<cv::Mat> input_channels;
  this->WrapInputLayer(&input_channels);
  this->Preprocess(_img, &input_channels);

  net_->ForwardPrefilled();

  /** Visualize the output of the first filter */

  // Store blob feature of layer fc7
  const boost::shared_ptr<caffe::Blob<float>> bf = net_->blob_by_name("fc7");
  
  std::vector<int> shape = bf->shape();
  // Shape can be size 4 or 2 (4 usually in convolution layers, 2 towards the end)
  // [N_batch, layer_depth, height, width]
  // [N_batch, layer_depth]

  // IN THIS CASE I AM CHOOSING THE FEATURE WITH SIZE 4096 WITH SHAPE SIZE 2
  const float* data= bf->cpu_data();
  std::vector<float> feature;
  int N = shape[1];
  for( int i = 0; i < N; ++i ) {
    feature.push_back( *data );
    data++;
  }
  
  return feature;

  /*
  if( shape.size() == 4 ) {
    for( int k = 0; k < shape[1]; ++k ) {
      
      cv::Mat m( shape[2], shape[3], CV_32FC1 );
      for( int j = 0; j < shape[2]; ++j ) {
	for( int i = 0; i < shape[3]; ++i ) {
	  m.at<float>(j,i) = *data;
	  data++;
	} 
      }
      ms.push_back( m );
    }
  }

  // Store 
  for( int i = 0; i < ms.size(); ++i ) {
    char name[50];
    sprintf(name, "m%d.png", i );
    double minVal, maxVal; cv::Point minLoc, maxLoc;
    cv::minMaxLoc( ms[i], &minVal, &maxVal, &minLoc, &maxLoc );
    printf("Min and max values: %f %f \n", minVal, maxVal );
    cv::Mat mn( ms[i].rows, ms[i].cols, CV_8UC1 );
    for( int j = 0; j < ms[i].rows; ++j ) {
	for( int i = 0; i < ms[i].cols; ++i ) {
	  mn.at<uchar>(j,i) = (uchar)floor(255.0*( (double)(ms[i].at<float>(j,i)) - minVal ) / (maxVal - minVal) );
	  data++;
	} 
      } 
    cv::imwrite( name, mn );
  }

  */
  
}


std::vector<float> Classifier::Predict( const cv::Mat &_img ) {

  caffe::Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  this->WrapInputLayer(&input_channels);
  this->Preprocess(_img, &input_channels);

  net_->ForwardPrefilled();
  

  /* Copy the output layer to a std::vector */
  caffe::Blob<float>* output_layer = net_->output_blobs()[0];
  const float* begin = output_layer->cpu_data();
  const float* end = begin + output_layer->channels();
  return std::vector<float>(begin, end);

}

void Classifier::WrapInputLayer( std::vector<cv::Mat>* _input_channels ) {
  caffe::Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    _input_channels->push_back(channel);
    input_data += width * height;
  }

}

void Classifier::Preprocess( const cv::Mat &img,
			     std::vector<cv::Mat>* input_channels ) {

  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, CV_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, CV_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, CV_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, CV_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";


}

