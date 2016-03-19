
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>


template<typename PointT>
void getBoundingBoxTable( const typename pcl::PointCloud<PointT>::Ptr _cloud,
			  double coeffs[4],
			  double _dim[3],
			  double _trans[3],
			  double _rot[3] ) {
  
  // Get centroid
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid( *_cloud, centroid );
  _trans[0] = centroid(0);
  _trans[1] = centroid(1); 
  _trans[2] = centroid(2);

  // Project to plane
  Eigen::VectorXd planeCoeffs(4); planeCoeffs << coeffs[0], coeffs[1], coeffs[2], coeffs[3];
  projectToPlane<PointT>( _cloud, planeCoeffs );
  // Get eigenvalues of the projected guys
  Eigen::Vector3d projCenter; Eigen::Vector3d ea, eb;
  getInfoFromProjectedCloud<PointT>( _cloud, 0.01, projCenter, ea, eb );

  // Rotation matrix: Normal from plane, ea, and eb
  ea.normalize(); eb.normalize();
  Eigen::Vector3d ec; ec << coeffs[0], coeffs[1], coeffs[2]; ec.normalize();
  Eigen::Matrix3d rot; rot.col(0) = ea; rot.col(2) = ec; rot.col(1) = ea.cross(ec);

  // Get the bounding box oriented in this way
  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  transf.block(0,3,3,1) << (float)centroid(0), (float)centroid(1), (float)centroid(2);
  transf.block(0,0,3,3) = rot.cast<float>();

  Eigen::Matrix4f tinv; tinv = transf.inverse();
  typename pcl::PointCloud<PointT>::Ptr cloud_temp( new pcl::PointCloud<PointT>() );
  pcl::transformPointCloud( *_cloud, *cloud_temp, tinv );

  // Get maximum and minimum
  PointT minPt; PointT maxPt;
  pcl::getMinMax3D( *cloud_temp, minPt, maxPt );
  double da, db, dc, dmax;
  da = ( maxPt.x - minPt.x ) / 2.0;
  db = ( maxPt.y - minPt.y ) / 2.0;
  dc = ( maxPt.z - minPt.z ) / 2.0;

  // Set z to be the biggest
  Eigen::Vector3d temp;
  if( da >= db && da >= dc ) { 
    temp = ec; ec = ea; ea = temp; eb = ec.cross(ea); 
    _dim[0] = dc; _dim[1] = db; _dim[2] = da;
  }
  if( db >= da && db >= dc ) { 
    temp = ec; ec = eb; eb = temp; ea = eb.cross(ec); 
    _dim[0] = da; _dim[1] = dc; _dim[2] = db;
  }
  if( dc >= da && dc >= db ) {
  _dim[0] = da;  _dim[1] = db;  _dim[2] = dc;  
  }

  Eigen::Matrix3d rot_2;
  rot_2.col(0) = ea; rot_2.col(1) = eb; rot_2.col(2) = ec;
  Eigen::Vector3d rpy = rot_2.eulerAngles(2,1,0);
  
  _rot[0] = (double)rpy(2);
  _rot[1] = (double)rpy(1);
  _rot[2] = (double)rpy(0);

}

/**
 * @brief ProjectToPlane
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr projectToPlane( typename pcl::PointCloud<PointT>::Ptr _cloud,
						      Eigen::VectorXd _planeCoeffs ) { 
  
  typename pcl::PointCloud<PointT>::Ptr projected( new pcl::PointCloud<PointT>() );

  
  // 1. Project and store
  typename pcl::PointCloud<PointT>::iterator it;
  PointT p; double a;
  
  // Plane equation: c0*x + c1*y + c2*z + c3 = 0
  // Original 3d point P will be projected (P') into plane with normal [c0, c1, c2]
  // P' = -(c3 + c0*x + c1*y + c2*z) / (c0^2 + c1^2 + c2^2) 
  for( it = _cloud->begin(); it != _cloud->end(); ++it ) {

    p = (*it);
    a = -( _planeCoeffs(3) + _planeCoeffs(0)*p.x + _planeCoeffs(1)*p.y + _planeCoeffs(2)*p.z );
    
    PointT pp;
    pp.x = p.x + _planeCoeffs(0)*a;
    pp.y = p.y + _planeCoeffs(1)*a;
    pp.z = p.z + _planeCoeffs(2)*a;
    
    projected->points.push_back( pp );
  }
  
  projected->height = 1;
  projected->width = projected->points.size();

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
				Eigen::Vector3d &_eb ) {

  pcl::PCA<PointT> pca;
  pca.setInputCloud( _cloud );
  Eigen::Vector3f eval = pca.getEigenValues();
  Eigen::Matrix3f evec = pca.getEigenVectors();

  // Calculate the centroid with the voxelized version of the projected cloud on the table
  // (otherwise the center is too influenced by the "front points" and might not
  // use the top information of the cloud, if available
  Eigen::Vector4d c;
  
  typename pcl::PointCloud<PointT>::Ptr projected_voxelized( new pcl::PointCloud<PointT>() );
  
  typename pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (_cloud);
  sor.setLeafSize ( (float)_voxel_size, (float)_voxel_size, (float)_voxel_size );
  sor.filter (*projected_voxelized);
  pcl::compute3DCentroid( *projected_voxelized, c );
    
  _center << c(0), c(1), c(2);
  _ea << (double) evec(0,0), (double) evec(1,0), (double) evec(2,0);
  _eb << (double) evec(0,1), (double) evec(1,1), (double) evec(2,1);

}

