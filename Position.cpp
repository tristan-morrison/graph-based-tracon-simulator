#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>

#include "Position.h"
#include "trigMath.h"
#include "constants.h"
#include "vectorMath.h"

Position::Position() {
  latitude = 0;
  longitude = 0;
  altitude = 0;
}

Position::Position(const Position &pos) {
  latitude = pos.latitude;
  longitude = pos.longitude;
  altitude = pos.altitude;
}

Position::Position(double myLatitude, double myLongitude) {
  latitude = myLatitude;
  longitude = myLongitude;
  altitude = 0;
}

Position::Position(double myLatitude, double myLongitude, int myAltitude) {
  latitude = myLatitude;
  longitude = myLongitude;
  altitude = myAltitude;
}

double Position::distanceTo(Position toPos) {
  const double R = GEO_CONSTANTS.EARTH_RADIUS_NM;
  const double phi_1 = TrigMath::degreesToRadians(latitude);
  const double phi_2 = TrigMath::degreesToRadians(toPos.latitude);
  const double delta_phi = TrigMath::degreesToRadians(toPos.latitude - latitude);
  const double delta_lambda = TrigMath::degreesToRadians(toPos.longitude - longitude);
  const double a = sin(delta_phi / 2) * sin(delta_phi / 2) + cos(phi_1) * cos(phi_2) * sin(delta_lambda / 2) * sin(delta_lambda / 2);
  const double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  const double d = R * c;

  return d;
}

double Position::bearingTo(Position pos) {
    const double phi_1 = TrigMath::degreesToRadians(latitude);
    const double phi_2 = TrigMath::degreesToRadians(pos.latitude);
    const double delta_lambda = TrigMath::degreesToRadians(pos.longitude - longitude);
    const double y = sin(delta_lambda) * cos(phi_2);
    const double x = cos(phi_1) * sin(phi_2) - sin(phi_1) * cos(phi_2) * cos(delta_lambda);
    const double theta = -atan2(y, x);

    // cout << "theta: " << theta << endl;

    return TrigMath::normalize_radians(theta - GEO_CONSTANTS.MAGNETIC_DECLINATION);

    // const float y = sin(pos.longitude - longitude) * cos(pos.latitude);
    // const float x = cos(latitude)*sin(pos.latitude) - sin(latitude)*cos(pos.latitude)*cos(pos.longitude - longitude);
    // float bearing = -atan2(y, x);
    // bearing = TrigMath::normalize_radians(bearing);
    // bearing = TrigMath::radiansToDegrees(bearing);

    // return bearing;

    // return VectorMath::vec2Radial(VectorMath::vecSub(getxyVec(), pos.getxyVec()));
}

void Position::setCoordinatesByBearingAndDistance(double bearing, double distance) {
    const int R = GEO_CONSTANTS.EARTH_RADIUS_NM;
    const double theta = bearing + double(GEO_CONSTANTS.MAGNETIC_DECLINATION);    // true bearing, in radians
    const double d = distance;
    const double smallDelta = d / double(R);    // angular distance, in earth laps
    const double phi_1 = TrigMath::degreesToRadians(latitude);
    const double lambda_1 = TrigMath::degreesToRadians(longitude);
    const double phi_2 = asin(sin(phi_1) * cos(smallDelta) + cos(phi_1) * sin(smallDelta) * cos(theta));
    const double lambda_2 = lambda_1 + atan2(sin(theta) * sin(smallDelta) * cos(phi_1), cos(smallDelta) - sin(phi_1) * sin(phi_2));
    const double lat = TrigMath::radiansToDegrees(phi_2);
    const double lon = TrigMath::radiansToDegrees(lambda_2);

    setCoordinatesLatLon(lat, lon);
}

void Position::setCoordinatesLatLon(double lat, double lon) {
  latitude = lat;
  longitude = lon;
}

bool Position::isEquivalentTo(Position pos) {
  if (latitude == pos.latitude && longitude == pos.longitude) {
    return true;
  } else {
    return false;
  }
}

vector<double> Position::getxyVec () {
  vector<double> resultVec(2, 0);
  resultVec[0] = longitude;
  resultVec[1] = latitude;
  return resultVec;
}
