/**
 * @file store_images_preprocessed.cpp
 * @brief Store images to be used with VGG features and SVM on top of it
 */
#include <sstream>
#include <algorithm>
#include <boost/scoped_ptr.hpp>
#include <chrono>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <caffe/caffe.hpp>
#include <caffe/util/io.hpp>
#include <caffe/util/db.hpp>

char* gList_file("/home/ana/Desktop/Crichton_data/training_images.txt");
char* gDb_name_train("Crichton_data_224_brute_resize_train");
char* gDb_name_val("Crichton_data_224_brute_resize_val");

// GoogleNet needs 224x224. AlexNet is happy with 227
cv::Size gInputGeometry_size( 224, 224 );
double val_ratio = 0.1;


cv::Mat preprocess_img( cv::Mat &_img );


/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  int c;
  while( (c=getopt(argc, argv,"f:") ) != -1 ) {
    switch(c) {
      /** List of file/label's */
    case 'f': {
      gList_file = optarg;
    } break;
      
    }
  } // end while
  
  // Preprocessing
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();  
  
  // Read list file
  std::map<int, std::vector<std::string> > full_data;
  
  std::ifstream infile( gList_file );
  std::string filename; int label;
  
  // Get the list of pairs image/label
  while( infile >> filename >> label ) {
    full_data[label].push_back(filename);
  }
  printf("Size of full data: %d \n", full_data.size() );
  
  std::map<int, std::vector<std::string> >::iterator it;
  for( it = full_data.begin(); it != full_data.end(); ++it ) {   
    printf("* Images with label [%d]: size: %d \n", it->first, it->second.size() );
  }
  
  // Get databases ready to store the images
  boost::scoped_ptr<caffe::db::DB> db_train( caffe::db::GetDB("lmdb") );
  boost::scoped_ptr<caffe::db::DB> db_val( caffe::db::GetDB("lmdb") );
  
  db_train->Open( gDb_name_train, caffe::db::NEW );
  db_val->Open( gDb_name_val, caffe::db::NEW );
  
  boost::scoped_ptr<caffe::db::Transaction> txn_train( db_train->NewTransaction() );
  boost::scoped_ptr<caffe::db::Transaction> txn_val( db_val->NewTransaction() );
  
  // Separate them in groups 
  
  // Select randomly a group of these to be validation
  int count_val = 0; int count_train = 0;
  const int kMaxKeyLength = 256;
  char key_cstr[kMaxKeyLength];

  for( it = full_data.begin(); it != full_data.end(); ++it ) {
    //std::shuffle( it->second.begin(), it->second.end(), std::default_random_engine(seed) );
    
    // Select the first x% for validation and the rest for training
    int N = (int)( it->second.size()*val_ratio );
    for( int i = 0; i < it->second.size(); ++i ) {
      cv::Mat img = cv::imread( it->second[i], -1 );
      cv::Mat img_proc = preprocess_img( img );
      caffe::Datum datum;
      // Store it with its label
      std::vector<uchar> buf;
      cv::imencode(".png", img_proc, buf);
      datum.set_data(std::string(reinterpret_cast<char*>(&buf[0]),
				  buf.size()));
      datum.set_label( it->first );
      datum.set_encoded(true);

      // Store in validation
      if( i < N ) {
	int length = snprintf( key_cstr, kMaxKeyLength, "%08d_%s", count_val,
			       it->second[i].c_str() );
	// Put in db
	std::string out;
	CHECK( datum.SerializeToString(&out) );
	txn_val->Put( std::string(key_cstr, length), out);
	txn_val->Commit();
	txn_val.reset( db_val->NewTransaction() );
	if( count_val % 20 == 0 ) { printf("Val images stored: %d \n", count_val); } 
	count_val++;
      } 
      // Store in testing
      else {
	int length = snprintf( key_cstr, kMaxKeyLength, "%08d_%s", count_train,
			       it->second[i].c_str() );
	// Put in db
	std::string out;
	CHECK( datum.SerializeToString(&out) );
	txn_train->Put( std::string(key_cstr, length), out);
	txn_train->Commit();
	txn_train.reset( db_train->NewTransaction() );
	if( count_train % 100 == 0 ) { printf("Train images stored: %d \n", count_train); } 
	count_train++;

      }

    } // for


  } // for full_data 
  
  printf("Count val: %d and train: %d \n", count_val, count_train );

} // end main

cv::Mat preprocess_img( cv::Mat &_img ) {
  
  /* Convert the input image to the input image format of the network. */
  
  cv::Mat sample;

  if( _img.channels() == 4 ) {
    cv::cvtColor( _img, sample, CV_BGRA2BGR );
  } else if ( _img.channels() == 1 ) {
    cv::cvtColor( _img, sample, CV_GRAY2BGR );
  } else
    sample = _img;

  cv::Mat sample_resized;
  if (sample.size() != gInputGeometry_size ) {
    cv::resize( sample, sample_resized, 
		gInputGeometry_size );
  } else {
    sample_resized = sample;
  }
  /*
  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);


  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
  */
  return sample_resized;
}
