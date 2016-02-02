/**
 * @file SQ_deformations.h
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include "perception/pointcloud_tools/sq_fitting/SQ_utils.h"

class SQ_deformations {
public:

  pcl::PointCloud<pcl::PointXYZ>::Ptr linear_tampering( const double &_a1,
							const double &_a2,
							const double &_a3,
							const double &_e1,
							const double &_e2,
							const double &_t );

  pcl::PointCloud<pcl::PointNormal>::Ptr tampering( const double &_a1,
						    const double &_a2,
						    const double &_a3,
						    const double &_e1,
						    const double &_e2,
						    const double &_t ) {
    
    pcl::PointCloud<pcl::PointNormal>::Ptr pn( new pcl::PointCloud<pcl::PointNormal>() );
    pcl::PointCloud<pcl::PointNormal>::Ptr tamp( new pcl::PointCloud<pcl::PointNormal>() );
    sampleSQ_uniform_pn( _a1, _a2, _a3, _e1, _e2, 25, pn );

    // Transform the points to bending points and normals
    pcl::PointCloud<pcl::PointNormal>::iterator it;
    double x,y,z,nx,ny,nz;
    double tx,ty,tz;
    Eigen::Matrix3d J;
    Eigen::Vector3d n;
    Eigen::Vector3d tn;
    pcl::PointNormal t;
    for( it = pn->begin(); it != pn->end(); ++it ) {
      x = (*it).x; y = (*it).y; z = (*it).z;
      n << (*it).normal_x, (*it).normal_y,  (*it).normal_z;

      tx = ((_t/_a3)*z +1)*x;
      ty = ((_t/_a3)*z +1)*y;
      tz = z;
      
      J << (_t/_a3)*z+1, 0, (_t/_a3)*x, 0,(_t/_a3)*z+1, (_t/_a3)*y,  0,0,1;
      tn = ( (J.inverse()).transpose() )*n;
      
      t.x = tx; t.y = ty; t.z = tz;
      t.normal_x = tn(0); t.normal_y = tn(1); t.normal_z = tn(2);
      tamp->points.push_back(t);
    }

    tamp->width = 1; tamp->height = tamp->points.size();
    // Store them both
    pcl::io::savePCDFileASCII( "original_tamp.pcd", *pn );
    pcl::io::savePCDFileASCII( "tamp_1.pcd", *tamp );

    return tamp;
  }

  
  pcl::PointCloud<pcl::PointNormal>::Ptr bending( const double &_a1,
						  const double &_a2,
						  const double &_a3,
						  const double &_e1,
						  const double &_e2,
						  const double &_R ) {
    
    pcl::PointCloud<pcl::PointNormal>::Ptr pn( new pcl::PointCloud<pcl::PointNormal>() );
    pcl::PointCloud<pcl::PointNormal>::Ptr bend( new pcl::PointCloud<pcl::PointNormal>() );
    sampleSQ_uniform_pn( _a1, _a2, _a3, _e1, _e2, 25, pn );

    // Transform the points to bending points and normals
    pcl::PointCloud<pcl::PointNormal>::iterator it;
    double x,y,z,nx,ny,nz;
    double bx,by,bz;
    Eigen::Matrix3d J;
    Eigen::Vector3d n;
    Eigen::Vector3d bn;
    pcl::PointNormal b;
    for( it = pn->begin(); it != pn->end(); ++it ) {
      x = (*it).x; y = (*it).y; z = (*it).z;
      n << (*it).normal_x, (*it).normal_y,  (*it).normal_z;

      bx = x;
      by = y - _R*(1.0-cos(z/_R));
      bz = _R*sin(z/_R);
      
      J << 1,0,0, 0,1,-sin(z/_R),  0,0,cos(z/_R);
      bn = ( (J.inverse()).transpose() )*n;
      
      b.x = bx; b.y = by; b.z = bz;
      b.normal_x = bn(0); b.normal_y = bn(1); b.normal_z = bn(2);
      bend->points.push_back(b);
    }

    bend->width = 1; bend->height = bend->points.size();
    // Store them both
    pcl::io::savePCDFileASCII( "original_1.pcd", *pn );
    pcl::io::savePCDFileASCII( "bend_1.pcd", *bend );

    return bend;
  }
  
protected:

};
