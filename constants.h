#include <stdlib.h>

#ifndef CONSTANTS_H
#define CONSTANTS_H

const struct AircraftPerformanceConstants {
  int speed = 250;

  float MINIMUM_SPEED_KN = 160;
  float MAXIMUM_SPEED_KN = 250;

  const int FINAL_APPROACH_SPEED_KN = 130;

  const int STANDARD_ACCELERATION = 7;

  const int STANDARD_DECELERATION = 4;

  // Maximum distance (in nautical miles) from a given fix at which an aircraft can be said to have passed by that fix
  // This relates to the concept of a Fly-over Fix
  const float MAXIMUM_DISTANCE_TO_PASS_WAYPOINT_NM = 0.5;

  // Maximum distance (in nautical miles) from a given fix at which an aircraft can be said to have "flown by" that fix
  // This relates to the concept of a Fly-by Fix
  const float MAXIMUM_DISTANCE_TO_FLY_BY_WAYPOINT_NM = 5;

  // Standard rate of turn, in radians per second
  // Equal to 3 degrees
  const double TURN_RATE = 0.0523599;

  // the lateral separation required between 2 aircraft, in nautical miles
  const int MINIMUM_LATERAL_SEPARATION_NM = 3;

  // the vertical separation required between 2 aircraft, in feet
  const int MINIMUM_VERTICAL_SEPARATION_FT = 1000;

  // specifies a standard rate climb using the rule of 3: 3 miles of travel should be allowed for every 1,000 feet of descent
  const int RATIO_MILES_TRAVELED_TO_FEET_OF_DESCENT = 3;

  // TODO: find a better-founded value for this number
  const int RATIO_MILES_TRAVELED_TO_FEET_OF_CLIMB = 3;

  const int DESCENT_RATE_LAST_THOUSAND_FEET_FPM = 500;


} AIRCRAFT_PERFORMANCE_CONSTANTS;

const struct AircraftPhysicsConstants {
  const float TRUE_AIRSPEED_INCREASE_FACTOR_PER_FOOT = 0.000016;
} AIRCRAFT_PHYSICS_CONSTANTS;

const struct PhysicsConstants {
  // downward acceleration due to gravity, in m/s^2
  const float GRAVITATIONAL_MAGNITUDE = 9.81;
} PHYSICS_CONSTANTS;

const struct GeoConstants {
  // TODO: add realistic value
  const float MAGNETIC_DECLINATION = 0;
  int EARTH_RADIUS_NM = 3440;
} GEO_CONSTANTS;

const struct UnitConvConstants {
  /*** Distance ***/
  // ratio of knots to meter/sec
  const float KN_MS = 0.51444444;
  // nautical miles per kilometer
  const float NM_KM = 1.852;
  // 1 nautical mile in feet
  const float NM_FT = 6076.12;

  /*** Time ***/
  const int MIN_HOUR = 60;
  const int SEC_MIN = 60;
  const int SEC_HOUR = 3600;
  const int MILLISEC_SEC = 1000;
  const int MILLISEC_MIN = 1000 * SEC_MIN;
} UNIT_CONVERSION_CONSTANTS;

#endif
