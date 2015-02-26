/**
 * @file testMethods.cpp
 */
#include <iostream>
#include <Eigen/Geometry>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/StdVector>

#include "levmar_eqs.h"

void load( std::vector<Eigen::Vector3d> &_Pk,
	   std::vector<Eigen::Vector3d> &_Pr,
	   int &_n );

void testData( std::vector<Eigen::Vector3d> &_Pk,
	       std::vector<Eigen::Vector3d> &_Pr );

bool svd_method( std::vector<Eigen::Vector3d> _Pk,
		 std::vector<Eigen::Vector3d> _Pr,
		 int _n,
		 Eigen::Isometry3d &_Tf );
bool icp_method( std::vector<Eigen::Vector3d> _Pk,
		 std::vector<Eigen::Vector3d> _Pr,
		 int _n,
		 Eigen::Isometry3d &_Tf );
bool lms_method( std::vector<Eigen::Vector3d> _Pk,
		 std::vector<Eigen::Vector3d> _Pr,
		 int _n,
		 Eigen::Isometry3d &_Tf );
bool ransac_method( std::vector<Eigen::Vector3d> _Pk,
		    std::vector<Eigen::Vector3d> _Pr,
		    int _n,
		    Eigen::Isometry3d &_Tf );

double getErr( std::vector<Eigen::Vector3d> _Pk,
	       std::vector<Eigen::Vector3d> _Pr,
	       Eigen::Isometry3d Tf,
	       bool verbose = false );

/**
 * @function main 
 */
int main( int argc, char* argv[] ) {

  // Load points
  std::vector<Eigen::Vector3d> Pk;
  std::vector<Eigen::Vector3d> Pr;
  int n;

  load( Pk, Pr, n );
  testData(Pk, Pr);

  // Run 4 methods
  Eigen::Isometry3d Tf_svd, Tf_icp, Tf_lms, Tf_ransac;

  svd_method( Pk, Pr, n, Tf_svd );
  icp_method( Pk, Pr, n, Tf_icp );
  lms_method( Pk, Pr, n, Tf_lms );
  ransac_method( Pk, Pr, n, Tf_ransac );

  std::cout << "TRANSFORM FROM SVD: \n"<< Tf_svd.matrix() << "\n avg. error: "<< getErr(Pk,Pr, Tf_svd, false )<<std::endl;
  std::cout << "TRANSFORM FROM ICP: \n"<< Tf_icp.matrix() << "\n avg. error: "<< getErr(Pk,Pr, Tf_icp, false )<< std::endl;
  std::cout << "TRANSFORM FROM LMS: \n"<< Tf_lms.matrix() << "\n avg. error: "<< getErr(Pk,Pr, Tf_lms, false )<< std::endl;
  std::cout << "TRANSFORM FROM RANSAC: \n"<< Tf_ransac.matrix() << "\n avg. error: "<< getErr(Pk,Pr, Tf_ransac, false )<< std::endl;

  return 1;

}

/**
 * @brief Hardcode from dataset taken
 */
void load(  std::vector<Eigen::Vector3d> &_Pk,
	    std::vector<Eigen::Vector3d> &_Pr,
	    int &_n ) {
  
  _n = 4;
  _Pk.resize(_n); 
  _Pr.resize(_n);
/*
_Pk[0] << -0.285675, 0.169231, 0.894000; _Pr[0] << 0.187429, 0.374712, 0.682116;
_Pk[1] << -0.265028, 0.139571, 0.903000; _Pr[1] << 0.170637, 0.343033, 0.688927;
_Pk[2] << -0.252991, 0.108882, 0.922000; _Pr[2] << 0.154053, 0.310005, 0.694107;
_Pk[3] << -0.234325, 0.078108, 0.937000; _Pr[3] << 0.137857, 0.275797, 0.697499;
_Pk[4] << -0.219612, 0.049912, 0.958000; _Pr[4] << 0.122160, 0.240602, 0.698932;
_Pk[5] << -0.201940, 0.020194, 0.969000; _Pr[5] << 0.107090, 0.204662, 0.698362;
_Pk[6] << -0.188161, -0.010357, 0.994000; _Pr[6] << 0.092790, 0.168180, 0.695624;
_Pk[7] << -0.174337, -0.035220, 1.014000; _Pr[7] << 0.079338, 0.131417, 0.690686;
_Pk[8] << -0.162865, -0.061527, 1.042000; _Pr[8] << 0.066760, 0.094639, 0.683512;
_Pk[9] << -0.149673, -0.086847, 1.064000; _Pr[9] << 0.055168, 0.058122, 0.674130;
_Pk[10] << -0.142103, -0.107998, 1.091000; _Pr[10] << 0.044682, 0.022127, 0.662613;
_Pk[11] << -0.130669, -0.132619, 1.123000; _Pr[11] << 0.035211, -0.013098, 0.648972;
_Pk[12] << -0.122145, -0.150178, 1.153000; _Pr[12] << 0.026911, -0.047295, 0.633432;
_Pk[13] << -0.113187, -0.168752, 1.185000; _Pr[13] << 0.019740, -0.080238, 0.616069;
_Pk[14] << -0.107170, -0.180718, 1.210000; _Pr[14] << 0.013702, -0.111755, 0.597084;
_Pk[15] << -0.099407, -0.207650, 1.272000; _Pr[15] << 0.005007, -0.169996, 0.555008;
*/
// Left
/*
_Pk[0] << 0.150828, 0.143645, 0.589000 ; _Pr[0] << 0.694284, 0.140661, -0.090792;
_Pk[1] << 0.111819, 0.166587, 0.655000 ; _Pr[1] << 0.763460, 0.103190, -0.128087;
_Pk[2] << 0.142931, 0.189622, 0.547000 ; _Pr[2] << 0.701993, 0.142634, -0.025510;
_Pk[3] << 0.104990, 0.218551, 0.615000 ; _Pr[3] << 0.757624, 0.108308, 0.031435;
*/

// Right

_Pk[0] << -0.121060, 0.147635, 0.565000; _Pr[0] << -0.702783, 0.145889, 0.026603;
_Pk[1] << -0.082341, 0.204797, 0.606000; _Pr[1] << -0.778582, 0.105860, 0.041846;
_Pk[2] << -0.117564, 0.093845, 0.592000 ; _Pr[2] << -0.706140, 0.151711, -0.039290;
_Pk[3] << -0.090004, 0.068964, 0.671000 ; _Pr[3] << -0.780080, 0.108061, -0.027313;

}

void testData( std::vector<Eigen::Vector3d> &_Pk,
	       std::vector<Eigen::Vector3d> &_Pr ) {

  double dk, dr;

  for( int i = 0; i < _Pk.size(); ++i ) {
    for( int j = i+1; j < _Pk.size(); ++j ) {
      dk = (_Pk[i] - _Pk[j]).norm();
      dr = (_Pr[i] - _Pr[j]).norm();
      std::cout << "["<<i<<","<<j<<"] Dk - Dr: "<< fabs(dk-dr) <<" Dk: "<< dk << " Dr: "<< dr << std::endl;
    }
  }


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

/** 
 * @function icp_method
 */
bool icp_method( std::vector<Eigen::Vector3d> _Pk,
		 std::vector<Eigen::Vector3d> _Pr,
		 int _n,
		 Eigen::Isometry3d &_Tf ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  

  // Fill in the CloudIn data
  cloud_in->width    = _n;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);

  cloud_out->width    = _n;
  cloud_out->height   = 1;
  cloud_out->is_dense = false;
  cloud_out->points.resize (cloud_out->width * cloud_out->height);


  for (size_t i = 0; i < _n; ++i) {
    cloud_in->points[i].x = _Pk[i](0);
    cloud_in->points[i].y = _Pk[i](1);
    cloud_in->points[i].z = _Pk[i](2);

    cloud_out->points[i].x = _Pr[i](0);
    cloud_out->points[i].y = _Pr[i](1);
    cloud_out->points[i].z = _Pr[i](2);
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "* ICP converged? :" << icp.hasConverged() << std::endl;
  std::cout << "* Fitness score: " << icp.getFitnessScore() << std::endl;

  Eigen::Matrix4f tf = icp.getFinalTransformation();
  
  for( int i = 0; i < 4; i++ ) {
    for( int j = 0; j < 4; j++ ) {
      _Tf.matrix()(i,j) = (double) tf(i,j);
    }
  }
  
  return true;
}

/** 
 * @function lms_method
 */
bool lms_method( std::vector<Eigen::Vector3d> _Pk,
		 std::vector<Eigen::Vector3d> _Pr,
		 int _n,
		 Eigen::Isometry3d &_Tf ) {

  int m = 6; // Parameters of transformation [R,t]
  double p[m]; // Parameter values
  double y[_n]; // Values we want to achieve

  double opts[LM_OPTS_SZ];
  double info[LM_INFO_SZ];
  
  opts[0] = LM_INIT_MU;
  opts[1] = 1E-15;
  opts[2] = 1E-15;
  opts[3] = 1E-20;
  opts[4] = LM_DIFF_DELTA;

  struct levmar_data data;
  data.xr = new double[_n];
  data.yr = new double[_n];
  data.zr = new double[_n];
  data.xk = new double[_n];
  data.yk = new double[_n];
  data.zk = new double[_n];
  data.num = _n;

  for( int i = 0; i < _n; ++i ) {
    data.xr[i] = _Pr[i](0);
    data.yr[i] = _Pr[i](1);
    data.zr[i] = _Pr[i](2);

    data.xk[i] = _Pk[i](0);
    data.yk[i] = _Pk[i](1);
    data.zk[i] = _Pk[i](2);
  }

  // Set value you want to achieve. We want zero
  for( int i = 0; i < _n; ++i ) {
    y[i] = 0;
  }

  // Initialize values for parameters p
  p[0] = 0.0; p[1] = 3.0; p[2] = 0.0;
  p[3] = 0.0; p[4] = 0.5; p[5] = 1.0;

  // Set limits
  
  double ub[m]; double lb[m];
  lb[0] = -3.14; ub[0] = 3.14;
  lb[1] = -3.14; ub[1] = 3.14;
  lb[2] = -3.14; ub[2] = 3.14;
  lb[3] = -2.0; ub[3] = 2.0; // tx
  lb[4] = 0.0; ub[4] = 2.0; // ty
  lb[5] = -2.0; ub[5] = 2.0; // tz
  
  int ret;
  ret = dlevmar_bc_der( levmar_fx, levmar_jac, 
			p, y, m, _n, 
			lb, ub,
			NULL, 5000, opts, info, // opts
			NULL, NULL, (void*)&data );

  std::cout << "* Levmar ends after " << info[5]<<" iterations " << std::endl;
  std::cout << "* Reason: "<< info[6]<< std::endl;
  std::cout << "* Sumsq: "<< info[1] << "["<<info[0]<<"]"<< std::endl;

  _Tf = Eigen::Isometry3d::Identity();
  _Tf.translation() << p[3], p[4], p[5];
  
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd( p[2], Eigen::Vector3d(0,0,1) )*
    Eigen::AngleAxisd( p[1], Eigen::Vector3d(0,1,0) )*
    Eigen::AngleAxisd( p[0], Eigen::Vector3d(1,0,0) );
  std::cout << " * Determinant: "<< rot.determinant() << std::endl;
  _Tf.linear() = rot;

  return true;
}

/** 
 * @function ransac_method
 */
bool ransac_method( std::vector<Eigen::Vector3d> _Pk,
		    std::vector<Eigen::Vector3d> _Pr,
		    int _n,
		    Eigen::Isometry3d &_Tf ) {

  std::vector<Eigen::Vector3d> Pks(4);
  std::vector<Eigen::Vector3d> Prs(4);

  std::vector<double> err;

  double minErr = 1000;
  double tempErr;
  Eigen::Isometry3d tempTf;
  Eigen::Isometry3d minTf;

  for( int i = 0; i < _n; ++i  ) {
    for(int j = i+1; j < _n; ++j ) {
      for(int k = j+1; k < _n; ++k ) {
	for(int m = k+1; m < _n; ++m ) {
	  Pks[0] = _Pk[i]; Prs[0] = _Pr[i];
	  Pks[1] = _Pk[j]; Prs[1] = _Pr[j];
	  Pks[2] = _Pk[k]; Prs[2] = _Pr[k];
	  Pks[3] = _Pk[m]; Prs[3] = _Pr[m];
	 
	  svd_method( Pks, Prs, 4, tempTf );
	  tempErr = getErr( _Pk, _Pr, tempTf);
	  err.push_back( tempErr );
	  if( tempErr < minErr ) { minErr = tempErr; minTf = tempTf; }
	} // for m	
      } // for k
    } // for j
  } // for i

  _Tf = minTf;

  return true;
}


/**
 * @function getErr
 */
double getErr( std::vector<Eigen::Vector3d> _Pk,
	       std::vector<Eigen::Vector3d> _Pr,
	       Eigen::Isometry3d _Tf, 
	       bool verbose ) {
  
  double err = 0;
  double eachErr;
  for( int i = 0; i < _Pk.size(); ++i ) {
    Eigen::Vector3d pt;
    pt = ( _Tf.linear()*_Pk[i] + _Tf.translation() );
    eachErr = (_Pr[i] - pt).norm(); 
    err +=  eachErr;    
    if( verbose ) {
      std::cout << "["<<i<<"] Pk: "<< _Pk[i].transpose() << 
	" Pr: "<< _Pr[i].transpose() << 
	" Prn: "<< pt.transpose() <<
	" err: "<< eachErr << std::endl;
    }
  }

  return err / (double)_Pk.size();
}

