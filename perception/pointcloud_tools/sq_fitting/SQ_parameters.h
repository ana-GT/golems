/**
 * @function SQ_parameters.h
 */
#pragma once

enum SQ_GEOMETRY_TYPES {
  REGULAR = 0,
  TAMPERED = 1,
  BENT = 2
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

