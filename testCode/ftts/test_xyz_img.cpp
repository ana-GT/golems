
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>


int main( int argc, char* argv[] ) {

cv::FileStorage fs(argv[1], cv::FileStorage::READ );

cv::Mat xyz_img;
fs["xyzMatrix"] >> xyz_img;

// Draw
printf("Width: %d, cols: %d \n", xyz_img.cols, xyz_img.rows);

cv::Mat dimg;
dimg = cv::Mat( xyz_img.rows, xyz_img.cols, CV_8UC1 );
for( int j = 0; j < xyz_img.rows; ++j ) {
for( int i = 0; i < xyz_img.cols; ++i ) {
 
cv::Vec3f p = xyz_img.at<cv::Vec3f>(j,i);
if( p(2) != p(2) ) { dimg.at<uchar>(j,i) = 255;}
 else { dimg.at<uchar>(j,i) = (uchar)( (p(2)/1.4)*255 ); }
}
}

// Save
cv::imwrite("dimg.png", dimg );

// Read the image data
printf("done!\n");

}
