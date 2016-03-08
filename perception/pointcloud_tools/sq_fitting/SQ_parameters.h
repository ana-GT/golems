/**
 * @function SQ_parameters.h
 */
#pragma once

#include "perception/msgs/perception_msgs.h"

enum SQ_GEOMETRY_TYPES {
  REGULAR = 0,
  TAMPERED = 1,
  BENT = 2,
  MULTIPLE = 3
};

enum SQ_FX_TYPES {
  SQ_FX_RADIAL = 0,
  SQ_FX_ICHIM = 1,
  SQ_FX_SOLINA = 2,
  SQ_FX_CHEVALIER = 3,
  SQ_FX_5 = 4,
  SQ_FX_6 = 5
};

/**
 * @struct SQ_parameters
 */
struct SQ_parameters {

  /** Dimensions: a,b,c */
  double dim[3];

  /** Coefficients: e1 & e2 */
  double e[2];

  /** Translation & Rotation */
  double trans[3];
  double rot[3];

  /** Tampering */
  double tamp;
  /** Bending */
  double R;
  double k; double alpha;

  /** Type */
  int type;
};

void copy_SQparam_msg( SQ_parameters params,
		       sns_sq_info  &_su );


/**
 * @structure levmar_data
 * @brief Contains the cloud's points info 
 */
struct levmar_data {
    double* x;
    double* y;
    double* z;
    int num;
};


