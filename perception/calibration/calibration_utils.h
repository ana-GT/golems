/**
 * @file calibration_utils.h
 * @brief Simple implementations of Umeyama and other calibration classics
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

bool umeyama( const std::vector<Eigen::Vector3d> &_A,
	      const std::vector<Eigen::Vector3d> &_B,
	      Eigen::Isometry3d &_Tf,
	      double &_c );

bool svd_method( std::vector<Eigen::Vector3d> _Pk,
		 std::vector<Eigen::Vector3d> _Pr,
		 int _n,
		 Eigen::Isometry3d &_Tf );
