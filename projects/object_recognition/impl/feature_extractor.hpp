#pragma once

#include <caffe/blob.hpp>
#include <caffe/common.hpp>
#include <caffe/net.hpp>
#include <caffe/util/db.hpp>
#include <caffe/util/io.hpp>

#include <unistd.h> // getopt



template<typename Dtype>
int feature_extraction_pipeline( int argc,
				 char** argv ) {

  ::google::InitGoogleLogging( argv[0] );
  
  char* pretrained_binary_proto = "/home/ana/Software/caffe/models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel";
  char* net_proto = "/home/ana/Software/caffe/examples/_temp/imagenet_val.prototxt";
  char* blob_name = "fc7";
  char* output_name = "/home/ana/Research/golems/bin/features";
  char* db_type = "lmdb";

  // We use CPU
  caffe::Caffe::set_mode( caffe::Caffe::CPU );

  // Argument reading
  int c;
  while( (c = getopt(argc, argv,"p:f:n:d:o:") ) != -1 ) {
    switch(c) {
      /** Pretrained model */
    case 'p': {
      pretrained_binary_proto = optarg;
    } break;
    case 'f': {
      net_proto = optarg;
    } break;
    case 'n': {
      blob_name = optarg;
    } break;
    case 'd': {
      db_type = optarg;
    } break;
    case 'o': {
      output_name = optarg;
    } break;
    } // switch
  } // while

  // Creating the network you will get features from
  std::shared_ptr<caffe::Net<Dtype> > net( new caffe::Net<Dtype>( net_proto, caffe::TEST ) );
  net->CopyTrainedLayersFrom( pretrained_binary_proto );
  
  // Open datasets to store
  std::shared_ptr<caffe::db::DB> feature_db( caffe::db::GetDB(db_type) );
  feature_db->Open( output_name, caffe::db::NEW );
  std::shared_ptr<caffe::db::Transaction> txn( feature_db->NewTransaction() );
  
  printf("Extracting features \n");
  caffe::Datum datum;
  const int kMaxKeyStrLength = 100;
  char key_str[kMaxKeyStrLength];
  std::vector< caffe::Blob<float>*> input_vec;
  int image_index = 0;

  int num_mini_batches = 10;
  for( int batch_index = 0; batch_index < num_mini_batches;
       ++batch_index ) {

    net->Forward( input_vec );
    printf("Size of input number: %d \n", net->num_inputs() );
    printf("Input vec size: %d \n", input_vec.size() );
    const boost::shared_ptr<caffe::Blob<Dtype> > feature_blob = net->blob_by_name( blob_name );
    int batch_size = feature_blob->num();
    int dim_features = feature_blob->count() / batch_size;
    
    printf("Batch size: %d dim features: %d, blob height: %d width: %d channels: %d \n", batch_size,
	   dim_features,
	   feature_blob->height(),
	   feature_blob->width(),
	   feature_blob->channels() );

    const Dtype* feature_blob_data;
    for( int n = 0; n < batch_size; ++n ) {
      datum.set_height( feature_blob->height() );
      datum.set_width( feature_blob->width() );
      datum.set_channels( feature_blob->channels() );
      datum.clear_data();
      datum.clear_float_data();
      feature_blob_data = feature_blob->cpu_data() + feature_blob->offset(n);
      printf("Offset [%d]: %d \n", n,  feature_blob->offset(n));
      for( int d = 0; d < dim_features; ++d ) {
	datum.add_float_data( feature_blob_data[d] );
      }
      int length = snprintf( key_str, kMaxKeyStrLength, "%010d",
			     image_index );
      std::string out;
      CHECK( datum.SerializeToString(&out) );
      txn->Put( std::string(key_str, length), out );
      image_index++;
      if( image_index % 1000 == 0 ) {
	txn->Commit();
	txn.reset( feature_db->NewTransaction() );
	printf("Extracted features of %d query images for feature blob \n", image_index, blob_name );
      }
    } // for n batch_size

  } // batch_index

  if( image_index % 1000 != 0 ) {
    txn->Commit();
  }

  feature_db->Close();


  printf("WE ARE DONE! \n");

}
