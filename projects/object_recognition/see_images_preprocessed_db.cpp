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

std::string gWindowName("See_database");
char* gDatabaseImg("/home/ana/Software/caffe/models/YCB_svm/YCB_train");

/**
 *
 */
int main(int argc, char** argv) {

  int c;
  while( (c=getopt( argc, argv, "d:h") ) != -1 ) {
    switch(c) {
    case 'd': {
      gDatabaseImg = optarg;
    } break;
    case 'h': {
      printf("Syntax: %s -d DATABASE_IMG \n",
	     argv[0] );
      return 1;
    } break;
    }
  }


  boost::scoped_ptr<caffe::db::DB> db(caffe::db::GetDB("lmdb"));
  db->Open( gDatabaseImg, caffe::db::READ);
  
  boost::scoped_ptr<caffe::db::Cursor> cursor( db->NewCursor() );
  int counter = 0;
  
  while (cursor->valid()) {

    caffe::Datum datum;
    datum.ParseFromString(cursor->value());
    caffe::DecodeDatumNative( &datum );
    std::cout << "Label of datum ["<<counter<<"]: "<<datum.label() << std::endl;

    cv::Mat m = caffe::DecodeDatumToCVMatNative( datum );
    char* label = new char[5];
    sprintf( label, "%d", datum.label() );
    cv::putText( m, label, cv::Point(30, 122), 
		 cv::FONT_HERSHEY_SIMPLEX, 2, 
		 cv::Scalar(100,100,100),
		 2 );
    printf("Element %d label: %d \n", counter, datum.label() );
    cv::imshow( gWindowName, m );
    cv::waitKey(30);

    cursor->Next();
    counter++;
  }
  printf("Showed %d files \n", counter);

  return 0;
}
