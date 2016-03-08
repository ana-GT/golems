
#include "SQ_parameters.h"

void copy_SQparam_msg( SQ_parameters params,
		       sns_sq_info  &_su ) {
  for( int i = 0; i < 3; ++i ) {
    _su.dim[i] = params.dim[i];
    _su.trans[i] = params.trans[i];
    _su.rot[i] = params.rot[i];
  }

  _su.e[0] = params.e[0]; _su.e[1] = params.e[1];
  _su.tamp = params.tamp;
  _su.R = params.R;
  _su.k = params.k;
  _su.alpha = params.alpha;
  _su.type = params.type;
  

}
