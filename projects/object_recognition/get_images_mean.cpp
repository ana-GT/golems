/**
 * @brief Get a database of training images and outputs the mean image binary.proto
 */

#include <stdint.h>
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <boost/scoped_ptr.hpp>
#include <caffe/proto/caffe.pb.h>
#include <caffe/util/db.hpp>
#include <caffe/util/io.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

char* gMeanImageFile("/home/ana/Software/caffe/models/ASU_svm/mean_ASU.binaryproto");
char* gDatabaseImg("/home/ana/Software/caffe/models/ASU_svm/ASU_train");

/**
 *
 */
int main(int argc, char** argv) {

  int c;
  while( (c=getopt( argc, argv, "d:o:h") ) != -1 ) {
    switch(c) {
    case 'd': {
      gDatabaseImg = optarg;
    } break;
    case 'o': {
      gMeanImageFile = optarg;
    } break;
    case 'h': {
      printf("Syntax: %s -d DATABASE_IMG -o OUTPUT_MEAN_IMG \n",
	     argv[0] );
      return 1;
    } break;
    }
  }

  boost::scoped_ptr<caffe::db::DB> db(caffe::db::GetDB("lmdb"));
  db->Open( gDatabaseImg, caffe::db::READ);
  
  boost::scoped_ptr<caffe::db::Cursor> cursor( db->NewCursor() );
  int count = 0;
  caffe::BlobProto sum_blob;

  caffe::Datum datum;
  datum.ParseFromString(cursor->value());
  DecodeDatumNative(&datum);

  sum_blob.set_num(1);
  sum_blob.set_channels(datum.channels());
  sum_blob.set_height(datum.height());
  sum_blob.set_width(datum.width());
  const int data_size = datum.channels() * datum.height() * datum.width();
  int size_in_datum = std::max<int>(datum.data().size(),
                                    datum.float_data_size());
  
  for (int i = 0; i < size_in_datum; ++i) {
    sum_blob.add_data(0.);
  }
  
  printf( "Starting Iteration \n" );
  
  while (cursor->valid()) {

    caffe::Datum datum;
    datum.ParseFromString(cursor->value());
    caffe::DecodeDatumNative( &datum );
    
    const std::string& data = datum.data();
    size_in_datum = std::max<int>(datum.data().size(),
				  datum.float_data_size());
    CHECK_EQ(size_in_datum, data_size) << "Incorrect data field size " <<
      size_in_datum;

    if (data.size() != 0) {
      CHECK_EQ(data.size(), size_in_datum);
      for (int i = 0; i < size_in_datum; ++i) {
        sum_blob.set_data(i, sum_blob.data(i) + (uint8_t)data[i]);
      }
    } else {
      CHECK_EQ(datum.float_data_size(), size_in_datum);
      for (int i = 0; i < size_in_datum; ++i) {
        sum_blob.set_data(i, sum_blob.data(i) +
			  static_cast<float>(datum.float_data(i)));
      }
    }
    
    ++count;
    if (count % 100 == 0) {
      printf( "Processed %d files. \n", count );
    }
    cursor->Next();
  } // while end

  if (count % 100 != 0) {
    printf( "Processed %d files. \n", count );
  }

  for (int i = 0; i < sum_blob.data_size(); ++i) {
    sum_blob.set_data(i, sum_blob.data(i) / count);
  }

  // Write to disk
  printf( "Write to %s \n", gMeanImageFile );
  caffe::WriteProtoToBinaryFile(sum_blob, gMeanImageFile );
  
  const int channels = sum_blob.channels();
  const int dim = sum_blob.height() * sum_blob.width();
  std::vector<float> mean_values(channels, 0.0);
  
  printf("Number of channels: %d \n", channels );
  for (int c = 0; c < channels; ++c) {
    for (int i = 0; i < dim; ++i) {
      mean_values[c] += sum_blob.data(dim * c + i);
    }
    printf( "mean_value channel [%d]: %f \n",  mean_values[c] / dim );
  }

  return 0;
}
