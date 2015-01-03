/**
 * @file calibration_utils.cpp
 */
#include "calibration_utils.h"
#include <iostream>
#include <stdio.h>
#include <Eigen/Geometry>

/**
 * @function umeyama
 * @brief _Bi = Tf*_A
 */
bool umeyama( const std::vector<Eigen::Vector3d> &_A,
	      const std::vector<Eigen::Vector3d> &_B,
	      Eigen::Isometry3d &_Tf,
	      double &_c ) {

  if( _A.size() != _B.size() ) {
    printf("\t [ERROR-Umeyama] Set of points have different sizes.\n");
    return false;
  }

  int n = _A.size();
  
  Eigen::Vector3d ux; ux << 0,0,0;
  Eigen::Vector3d uy; uy << 0,0,0;
  Eigen::Matrix3d sigma_xy; sigma_xy << 0,0,0, 0,0,0, 0,0,0;
  
  // Calculate mean
  for( int i = 0; i < _A.size(); ++i ) {
    ux += _A[i];
    uy += _B[i];
  }

  // Calculate covariance
  for( int i = 0; i < _A.size(); ++i ) {
    sigma_xy += (_B[i] - uy )*((_A[i] - ux).transpose());
  }
  sigma_xy /= (double) n;

  // Calculate gamma2
  double gamma2x = 0;
  
  for( int i = 0; i < _A.size(); ++i ) {
    gamma2x += (_A[i] - ux ).transpose() * (_A[i] - ux );
  }
  gamma2x = gamma2x / (double)n;
  
  // 4. Compute the singular value decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd( sigma_xy, Eigen::ComputeThinU | Eigen::ComputeThinV );
  Eigen::Matrix3d U; Eigen::Matrix3d V; Eigen::Matrix3d D;
  U = svd.matrixU(); V = svd.matrixV();
  Eigen::Vector3d singVals = svd.singularValues();
  D << singVals(0),0,0, 0,singVals(1),0, 0,0,singVals(2);
  
  Eigen::Matrix3d S; S = Eigen::Matrix3d::Identity();
  if( sigma_xy.determinant() < 0 ) { S(2,2) = -1; }
  

  // Rotation
  Eigen::Matrix3d R; Eigen::Vector3d t; double c;
  R = U*S*V.transpose();

  // Scale
  Eigen::Matrix3d DS = D*S;
  c = (1.0/gamma2x)*( DS(0,0) + DS(1,1) + DS(2,2) );

    // Translation
  t = uy - c*R*ux;

  std::cout << "Rotation: \n"<< R << std::endl;
  std::cout << "Translation: "<< t.transpose() << std::endl;
  std::cout << "Scale: \n" << c << std::endl;

  _Tf.setIdentity();
  _Tf.linear() = R;
  _Tf.translation() = t;
  _c = c;
  
}


/**
 * @function svd_method
 */
bool svd_method( std::vector<Eigen::Vector3d> _Pk,
		 std::vector<Eigen::Vector3d> _Pr,
		 int _n,
		 Eigen::Isometry3d &_Tf ) {

  if( _n < 4 ) {
    std::cout<<"\t * [SVD] At least 4 points \n" << std::endl;
    return false;
  }
  
  // 1. Compute the weighted centroids of both kinect and world sets
  // (all points same weight in our case)
  Eigen::Vector3d xm; xm << 0, 0, 0; // Points from Kinect
  Eigen::Vector3d ym; ym << 0, 0, 0; // Points from kinematics
  
  for( int i = 0; i < _n; ++i ) {    
    xm += _Pk[i];
    ym += _Pr[i];
  }
  
  xm = xm / _n; ym = ym / _n;
  
  // 2. Compute centered vectors
  Eigen::MatrixXd X(3,_n);
  Eigen::MatrixXd Y(3,_n);	

  for( int i = 0; i < _n; ++i ) {
    X.col(i) = _Pk[i] - xm;
    Y.col(i) = _Pr[i] - ym;
  }
  
  // 3. Compute the 3x3 covariance matrix
  Eigen::Matrix3d S;
  S = X*(Y.transpose()); // X*W*Yt -> W is identity


  // 4. Compute the singular value decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd( S, Eigen::ComputeThinU | Eigen::ComputeThinV );
  Eigen::Matrix3d U; Eigen::Matrix3d V;
  U = svd.matrixU(); V = svd.matrixV();
  
  Eigen::Matrix3d temp; temp = V*(U.transpose()); 
  Eigen::Matrix3d M; M.setIdentity(); M(2,2) = temp.determinant();
  if( M(2,2) == -1 ) {
    //std::cout << "[SVD] Was reflection (det: "<< M(2,2) <<")"<<std::endl;
  }
  Eigen::Matrix3d Rot;
  Rot = V*M*(U.transpose());

  Eigen::Vector3d trans;
  trans = ym - Rot*xm;

  _Tf.setIdentity();
  _Tf.linear() = Rot;
  _Tf.translation() = trans;

  return true;
}
