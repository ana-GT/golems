
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>

template<typename PointT>
void getBoundingBoxTable( const typename pcl::PointCloud<PointT>::Ptr _cloud,
			    double coeffs[4],
			    double _dim[3],
			    double _trans[3],
			    double _rot[3] ) {


// Project to plane
Eigen::VectorXd planeCoeffs(4); planeCoeffs << coeffs[0], coeffs[1], coeffs[2], coeffs[3];
typename pcl::PointCloud<PointT>::Ptr projected( new typename pcl::PointCloud<PointT> );

double dc_min, dc_max;
Eigen::Vector3d N; N << planeCoeffs(0), planeCoeffs(1), planeCoeffs(2); N.normalize();  

projected = projectToPlane<PointT>( _cloud, planeCoeffs, dc_min, dc_max );

// Get eigenvalues of the projected guys
Eigen::Vector3d projCenter; Eigen::Vector3d ea, eb, ec;
double da, db, dc;
getInfoFromProjectedCloud<PointT>( projected, 0.005, projCenter, 
				     ea, eb, da, db );
dc = (dc_max - dc_min)*0.5;
  
Eigen::Matrix3d Rot; 
Rot.col(0) = ea; Rot.col(2) = N; Rot.col(1) = (Rot.col(2)).cross(Rot.col(0));

// Set trans
Eigen::Vector3d tc; tc = projCenter +  N*(dc_max + dc_min)*0.5;
for(int i = 0; i < 3; ++i ) { _trans[i] = tc(i); }

// Set z to be the biggest
Eigen::Vector3d temp;
if( da >= db && da >= dc ) {
ea = -Rot.col(2); eb = Rot.col(1); ec = Rot.col(0);
_dim[0] = dc; _dim[1] = db; _dim[2] = da;
}
if( db >= da && db >= dc ) { 
ea = Rot.col(0); eb = -Rot.col(2); ec = Rot.col(1);
_dim[0] = da; _dim[1] = dc; _dim[2] = db;
}
if( dc >= da && dc >= db ) {
ea = Rot.col(0); eb = Rot.col(1); ec = Rot.col(2);
_dim[0] = da;  _dim[1] = db;  _dim[2] = dc;  
}

Rot.col(0) = ea; Rot.col(1) = eb; Rot.col(2) = ec;
Eigen::Vector3d rpy = Rot.eulerAngles(2,1,0);

_rot[0] = (double)rpy(2);
_rot[1] = (double)rpy(1);
_rot[2] = (double)rpy(0);

}

/**
 * @brief ProjectToPlane
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr projectToPlane( typename pcl::PointCloud<PointT>::Ptr _cloud,
						      Eigen::VectorXd _planeCoeffs,
						      double &_dmin, double &_dmax ) { 
  
  typename pcl::PointCloud<PointT>::Ptr projected( new pcl::PointCloud<PointT>() );

  
  // 1. Project and store
  typename pcl::PointCloud<PointT>::iterator it;
  PointT p; double a; double d;
  double dmin, dmax;
  double an = sqrt( _planeCoeffs(0)*_planeCoeffs(0) +
		    _planeCoeffs(1)*_planeCoeffs(1) +
		    _planeCoeffs(2)*_planeCoeffs(2) );

  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  dmin = 10; dmax = 0;

  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( _planeCoeffs(3) + _planeCoeffs(0)*p.x + _planeCoeffs(1)*p.y + _planeCoeffs(2)*p.z );
    
    d = fabs(a/an);

    PointT pp;
    pp.x = p.x + _planeCoeffs(0)*a;
    pp.y = p.y + _planeCoeffs(1)*a;
    pp.z = p.z + _planeCoeffs(2)*a;
    
    projected->points.push_back( pp );

    if( d < dmin ) { dmin = d; } if( d > dmax ) { dmax = d; }     
  }
  
  projected->height = 1;
  projected->width = projected->points.size();

  _dmin = dmin;
  _dmax = dmax;

  return projected;
}

/**
 * @brief ea: Bigger, eb: Smaller
 */
template<typename PointT>
void getInfoFromProjectedCloud( typename pcl::PointCloud<PointT>::Ptr _cloud,
				double _voxel_size,
				Eigen::Vector3d &_center,
				Eigen::Vector3d &_ea,
				Eigen::Vector3d &_eb,
				double &_dima, double &_dimb ) {

  // Calculate the centroid with the voxelized version of the projected cloud on the table
  // (otherwise the center is too influenced by the "front points" and might not
  // use the top information of the cloud, if available
  pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
  PointT min_OBB, max_OBB, pos_OBB; Eigen::Matrix3f rot_OBB;
  double dx, dy, dz; 
  Eigen::Vector3f ex, ey, ez;
  
  typename pcl::PointCloud<PointT>::Ptr projected_voxelized( new pcl::PointCloud<PointT>() );
  
  typename pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (_cloud);
  sor.setLeafSize ( (float)_voxel_size, (float)_voxel_size, (float)_voxel_size );
  sor.filter (*projected_voxelized);

  feature_extractor.setInputCloud(projected_voxelized);
  feature_extractor.compute();
  feature_extractor.getOBB( min_OBB, max_OBB, pos_OBB, rot_OBB );
  feature_extractor.getEigenVectors( ex, ey, ez );
  Eigen::Vector3f cm;
  feature_extractor.getMassCenter( cm );
  
  dx = 0.5*(max_OBB.x - min_OBB.x);
  dy = 0.5*(max_OBB.y - min_OBB.y);
  dz = 0.5*(max_OBB.z - min_OBB.z);

  // Output
  _center = cm.cast<double>();
  //_center << pos_OBB.x, pos_OBB.y, pos_OBB.z;
_ea = ex.cast<double>() ; _eb = ey.cast<double>();
_dima = dx; _dimb = dy;
}

