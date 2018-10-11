#include <stdlib.h>

#include "AircraftMoment.h"
#include "Position.h"
#include "Fix.h"

AircraftMoment::AircraftMoment () {
  nextFix = nullptr, previousFix = nullptr;
  isAtDestination = false;
  nextFix_index = -1;
  previousFix_index = INT_MAX;
  arcLengthCoord = 0;
  heading = 0;
  altitude = 0;
  altitudeTrend = 0;
  speed = 170;
  groundSpeed = 0;
  groundTrack = 0;
  hasConflict = false;
}
