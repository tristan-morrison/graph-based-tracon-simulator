#include <stdlib.h>
#include <math.h>

#include "constants.h"

#ifndef UNIT_CONV_H
#define UNIT_CONV_H

namespace UnitConverters {

  /*** Distance ***/
  double kn_to_ms (double knots) {
    return knots * UNIT_CONVERSION_CONSTANTS.KN_MS;
  }

  double km_to_nm (double km) {
    return km / UNIT_CONVERSION_CONSTANTS.NM_KM;
  }

  /*** Time ***/
  double millisec_to_sec(double millisec) {
    return millisec / UNIT_CONVERSION_CONSTANTS.MILLISEC_SEC;
  }

  double sec_to_hour(double sec) {
    return sec / UNIT_CONVERSION_CONSTANTS.SEC_HOUR;
  }

  double millisec_to_hour(double millisec) {
    return sec_to_hour(millisec_to_sec(millisec));
  }

}

#endif
