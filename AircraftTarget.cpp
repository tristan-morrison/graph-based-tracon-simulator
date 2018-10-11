#include "AircraftTarget.h"
#include "constants.h"

AircraftTarget::AircraftTarget () {
  targetedHeading = 0;
  targetedAltitude = 0;
  targetedSpeed = AIRCRAFT_PERFORMANCE_CONSTANTS.MAXIMUM_SPEED_KN;
}
