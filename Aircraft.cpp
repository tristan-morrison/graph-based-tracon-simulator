#include <QDebug>

#include "SimpleRenderer.h"
#include "SimpleMarkerSymbol.h"

#include <stdlib.h>
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <climits>

#include "Aircraft.h"
#include "AircraftTarget.h"
#include "Position.h"
#include "Path.h"
#include "constants.h"
#include "VectorMath.h"
#include "trigMath.h"
#include "unitConverters.h"
#include "Conflict.h"
// #include "vectorMath.h"

using namespace std;

Aircraft::Aircraft () {
    path = new Path(0, 0);
    // float speed = 160;
    hasArrived = false;
    pointBuilder = new Esri::ArcGISRuntime::PointBuilder(Esri::ArcGISRuntime::SpatialReference::wgs84());
//    graphicsOverlay = new Esri::ArcGISRuntime::GraphicsOverlay(parent);
}

Aircraft::Aircraft (QObject * myParent) {
  parent = myParent;
  path = new Path(0, 0);
  // float speed = 160;
  hasArrived = false;
  pointBuilder = new Esri::ArcGISRuntime::PointBuilder(Esri::ArcGISRuntime::SpatialReference::wgs84());
  graphicsOverlay = new Esri::ArcGISRuntime::GraphicsOverlay(parent);
}

//Aircraft::Aircraft (Aircraft &aircraftToCopy) {
//    parent = nullptr;
//    path = new Path(*aircraftToCopy.path);
//    target = aircraftToCopy.target;
//    moment = aircraftToCopy.moment;
//    futurePositions = aircraftToCopy.futurePositions;
//    arrivalTime_zulu = aircraftToCopy.arrivalTime_zulu;
//    landingTime_ms = aircraftToCopy.landingTime_ms;
//    callsign = aircraftToCopy.callsign;
//    isAtDestination = aircraftToCopy.isAtDestination;
//    hasArrived = aircraftToCopy.hasArrived;
//    pointBuilder = new Esri::ArcGISRuntime::PointBuilder(Esri::ArcGISRuntime::SpatialReference::wgs84());
//    graphic = nullptr;
//    graphicsOverlay = new Esri::ArcGISRuntime::GraphicsOverlay();
//}

void Aircraft::setPath(Path* newPath) {
  path = newPath;

  moment.previousFix = path->origin;

  if (path->fixSequence.size() > 0) {
    moment.nextFix = path->fixSequence[moment.nextFix_index];
    moment.nextFix_index = 0;
    moment.previousFix_index = 0;
  } else {
    moment.previousFix_index = 0;
    moment.nextFix_index = INT_MAX;
    moment.nextFix = path->destination;
  }

}

void Aircraft::generateFuturePositions(Timer timer, int step) {
  AircraftMoment saveMoment = moment;
  AircraftTarget saveTarget = target;

  qDebug() << "currentPosition: " << moment.currentPosition.latitude << ", " << moment.currentPosition.longitude << endl;
  qDebug() << step << endl;

  const long arrivalTime_ms = timer.getMillisecondsOffsetFromStart(arrivalTime_zulu);
  const long remaindr = arrivalTime_ms % step;
  const int diff = step - remaindr;
  const int firstDeltaTime = step + diff;
  const long firstRecordTime = arrivalTime_ms + firstDeltaTime;

  qDebug() << firstDeltaTime << endl;
  qDebug() << firstRecordTime << endl;

  update(firstDeltaTime);
  FuturePosition * futPos = new FuturePosition(moment.currentPosition, moment.nextFix_index, moment.previousFix_index);
  futurePositions.insert(pair<long, FuturePosition*>(firstRecordTime, futPos));

  long deltaTime = step;
  long recordTime = firstRecordTime;
  while (!(moment.isAtDestination)) {
//     cout << "recordTime: " << recordTime << endl;
    recordTime += deltaTime;
    update(deltaTime);
    FuturePosition * futPos = new FuturePosition(moment.currentPosition, moment.nextFix_index, moment.previousFix_index);
    futurePositions.insert(pair<long, FuturePosition*>(recordTime, futPos));
  }
  landingTime_ms = recordTime;

  moment = saveMoment;
  target = saveTarget;

  // cout << "moment.nextFix: " << moment.nextFix->name << endl;
  // cout << "currentPosition: " << moment.currentPosition.latitude << ", " << moment.currentPosition.longitude << endl;


}

void Aircraft::update (long deltaTime) {
  updateTarget();

  if (moment.isAtDestination) {
      removeFromMap();
  }

  updateSpeedPhysics(deltaTime);
  updateTurnPhysics(deltaTime);
  updateAltitudePhysics(deltaTime);
  updateGroundSpeedPhysics(deltaTime);
  updatePointGraphic();

  // cout << "fixSequence len: " << path->fixSequence.size() << endl;
  // cout << "moment.nextFix_index: " << moment.nextFix_index << endl;
  // cout << "nextFix: " << moment.nextFix->name << endl;

}

void Aircraft::updateTarget () {
  updateTargetedHeading();
  updateTargetedAltitude();
  updateTargetedSpeed();
}

void Aircraft::updateTargetedHeading () {
  if (moment.isAtDestination) {
    return;
  }

  const Position nextFixPos = *(moment.nextFix->position);
  const double distanceToNextFix = moment.currentPosition.distanceTo(nextFixPos);
  const double headingToNextFix = moment.currentPosition.bearingTo(nextFixPos);
  const bool isTimeToStartTurning = distanceToNextFix < UnitConverters::km_to_nm(calculateTurnInitiaionDistance(nextFixPos));
  const bool closeToBeingOverNextFix = distanceToNextFix < AIRCRAFT_PERFORMANCE_CONSTANTS.MAXIMUM_DISTANCE_TO_PASS_WAYPOINT_NM;
  const bool closeEnoughToFlyByFix = distanceToNextFix < AIRCRAFT_PERFORMANCE_CONSTANTS.MAXIMUM_DISTANCE_TO_FLY_BY_WAYPOINT_NM;
  const bool shouldFlyByFix = closeEnoughToFlyByFix && isTimeToStartTurning;
  bool shouldMoveToNextFix = closeToBeingOverNextFix;

//  qDebug() << "nextFix: " << QString::fromStdString(moment.nextFix->name) << endl;
//  qDebug() << "destination: " << QString::fromStdString(path->destination->name) << endl;

  if (shouldMoveToNextFix) {
    qDebug() << QString::fromStdString(callsign) << "SHOULD MOVE TO NEXT FIX";
    if (moment.nextFix == path->destination) {
      moment.isAtDestination = true;
    }
    else if (moment.nextFix_index >= (path->fixSequence.size() - 1)) {
      moment.previousFix = moment.nextFix;
      moment.nextFix = path->destination;
      moment.previousFix_index = moment.nextFix_index;
      moment.nextFix_index = INT_MAX;
    } else {
      moment.previousFix = moment.nextFix;
      moment.previousFix_index = moment.nextFix_index;
      moment.nextFix_index++;
      moment.nextFix = path->fixSequence[moment.nextFix_index];
    }
    updateTargetedAltitude();
  }

  target.targetedHeading = headingToNextFix;

  if (isAtOrigin()) {
    moment.heading = headingToNextFix;
  }


  // if (isAtOrigin()) {
  //   cout << "IS AT ORIGIN!" << endl;
  //   nextFix = path->fixSequence[0];
  //   cout << "current pos lat: " << moment.currentPosition.latitude << endl;
  //   cout << "next fix lat: " << nextFix->position->latitude << endl;
  //   heading = moment.currentPosition.bearingTo(*(nextFix->position));
  // }
}

// myTargetedAltitude defaults to INT_MAX
void Aircraft::updateTargetedAltitude(int myTargetedAltitude) {
  if (myTargetedAltitude == INT_MAX) {
    myTargetedAltitude = moment.nextFix->altitude;
  }

  target.targetedAltitude = myTargetedAltitude;
}

// myTargetedSpeed defaults to NULL
void Aircraft::updateTargetedSpeed(int myTargetedSpeed) {
  // if no myTargetedSpeed paramter has been passed
  if (myTargetedSpeed == NULL) {
      // if the speed of the next fix is less than the currently targeted speed, accept the next fix's speed as targeted speed
      if (moment.nextFix->speed < target.targetedSpeed) {
          myTargetedSpeed = moment.nextFix->speed;
      } else {
          // if we're already going slower than the speed of the next fix, maintain the current targeted
          myTargetedSpeed = target.targetedSpeed;
      }
//      qDebug() << "myTargetedSpeed: " << myTargetedSpeed << endl;
  }

  if (myTargetedSpeed == INT_MAX) {
    myTargetedSpeed = AIRCRAFT_PERFORMANCE_CONSTANTS.FINAL_APPROACH_SPEED_KN;
  }

  target.targetedSpeed = myTargetedSpeed;
}

void Aircraft::updateAltitudePhysics (long deltaTime) {
  if (target.targetedAltitude == moment.altitude) {
    moment.altitudeTrend = 0;
  }
  else if (target.targetedAltitude < moment.altitude) {
    decreaseAltitude(deltaTime);
    moment.altitudeTrend = -1;
  } else if (target.targetedAltitude > moment.altitude) {
    increaseAltitude(deltaTime);
    moment.altitudeTrend = 1;
  }
}

void Aircraft::decreaseAltitude (long deltaTime) {
  const float altitude_diff = moment.altitude - target.targetedAltitude;

  // descent rate in feet per minute
  float descentRate_fpm;
  if (altitude_diff <= 1000) {
    // if aircraft is within 1000 feet of targeted altitude, rate of descent slows to a standard rate for last 1000 feet of descent
    descentRate_fpm = AIRCRAFT_PERFORMANCE_CONSTANTS.DESCENT_RATE_LAST_THOUSAND_FEET_FPM;
  } else {
    const float nm_per_min = moment.speed / float(UNIT_CONVERSION_CONSTANTS.MIN_HOUR);
    const float mins_per_three_miles = (1 / nm_per_min) * 3;
    // mins_per_three_miles = mins per thousand feet of descent
    descentRate_fpm = 1000 / mins_per_three_miles;
  }

  const float deltaTime_secs = deltaTime / double(UNIT_CONVERSION_CONSTANTS.MILLISEC_SEC);
  const float feetPerSecond = descentRate_fpm / double(UNIT_CONVERSION_CONSTANTS.SEC_MIN);
  const float feetDescended = feetPerSecond * deltaTime_secs;

  // scout << "Descent Rate (fpm): " <<
  cout << "Feet descended: " << feetDescended << endl;

  if (abs(altitude_diff) < feetDescended) {
    moment.altitude = target.targetedAltitude;
  } else {
    moment.altitude -= feetDescended;
  }
  moment.currentPosition.altitude = moment.altitude;

}

void Aircraft::increaseAltitude (long deltaTime) {
  const float altitude_diff = target.targetedAltitude - moment.altitude;

  // climb rate in feet per minute
  float climbRate_fpm;
  climbRate_fpm = (moment.speed / float(UNIT_CONVERSION_CONSTANTS.MIN_HOUR)) / AIRCRAFT_PERFORMANCE_CONSTANTS.RATIO_MILES_TRAVELED_TO_FEET_OF_CLIMB;

  const float feetPerSecond = climbRate_fpm * UNIT_CONVERSION_CONSTANTS.SEC_MIN;
  const float feetClimbed = feetPerSecond * deltaTime;

  if (abs(altitude_diff) < feetClimbed) {
    moment.altitude = target.targetedAltitude;
  } else {
    moment.altitude -= feetClimbed;
  }
  moment.currentPosition.altitude = moment.altitude;

}

void Aircraft::updateTurnPhysics (long deltaTime) {
  if (moment.heading == target.targetedHeading) {
    // no need for turning if target heading matches current heading
    return;
  }

  const float secondsElapsed = UnitConverters::millisec_to_sec(deltaTime); // TODO: game time
  const float angleDiff = TrigMath::angleOffset(target.targetedHeading, moment.heading);

  const float angleChange = AIRCRAFT_PERFORMANCE_CONSTANTS.TURN_RATE * secondsElapsed;

  // cout << "Angle Diff: " << angleDiff << endl;

  if (abs(angleDiff) <= angleChange) {
    // if difference in targeted and current headings is less than the radians of turn that can be covered in the time elapsed, then we have acheived our targeted moment.heading
    moment.heading = target.targetedHeading;
  } else if (angleDiff <= 0) {
    moment.heading = TrigMath::normalize_radians(moment.heading - angleChange);
  } else if (angleDiff > 0) {
    moment.heading = TrigMath::normalize_radians(moment.heading + angleChange);
  }
}

double Aircraft::calculateTurnInitiaionDistance (Position nextFixPos) {
  double currentHeading = moment.heading;
  const float nominalBankAngleDegrees = 25;
  const float mySpeed = UnitConverters::kn_to_ms(moment.speed);
  const float bankAngle = TrigMath::degreesToRadians(nominalBankAngleDegrees);
  Fix myNextFix = *(moment.nextFix);

  if (moment.nextFix == nullptr) {
    cout << "Next fix was null!!" << endl;
    return 0;
  }

  const double newNominalCourse = calculateNewNominalCourse(*(moment.nextFix->position), *(moment.previousFix->position));

  const double courseChange = calculateCourseChangeInRadians(currentHeading, newNominalCourse);

  const double turnInitiationDistance = calcTurnInitiationDistance(mySpeed, bankAngle, courseChange);

  // convert m to km
  return turnInitiationDistance / 1000;

}

double Aircraft::calculateNewNominalCourse (Position myNextFixPos, Position myCurrentFixPos) {

  double newNominalCourse = VectorMath::vec2Radial(VectorMath::vecSub(myNextFixPos.getxyVec(), myCurrentFixPos.getxyVec()));

  if (newNominalCourse < 0) {
    newNominalCourse += TrigMath::TAU;
  }

  return newNominalCourse;
}

double Aircraft::calculateCourseChangeInRadians (double currentHeading, double newNominalCourse) {
  double courseChange = abs(TrigMath::radiansToDegrees(currentHeading) - TrigMath::radiansToDegrees(newNominalCourse));

  if (courseChange > 180) {
    courseChange = 360 - courseChange;
  }

  return TrigMath::degreesToRadians(courseChange);
}

double Aircraft::calcTurnInitiationDistance (float speed, float bankAngle, double courseChange) {
  const double turnRadius = calcTurnRadius(speed, bankAngle);

  return turnRadius * tan(courseChange / 2) + speed;
}

double Aircraft::calcTurnRadius (float speed, float bankAngle) {
  return (speed * speed) / (PHYSICS_CONSTANTS.GRAVITATIONAL_MAGNITUDE * tan(bankAngle));
}

void Aircraft::updateSpeedPhysics (long deltaTime) {
  float speedChange = 0;
  const int differenceBetweenPresentAndTargetSpeeds = moment.speed - target.targetedSpeed;

  if (differenceBetweenPresentAndTargetSpeeds == 0) {
    return;
  }

  // std::cout << "updateSpeedPhysics: deltaTime = " << deltaTime << std::endl;
  const double deltaTime_secs = deltaTime / double(UNIT_CONVERSION_CONSTANTS.MILLISEC_SEC);
  // std::cout << "updateSpeedPhysics: deltaTime_secs = " << deltaTime_secs << std::endl;
  if (moment.speed > target.targetedSpeed) {
    speedChange = -AIRCRAFT_PERFORMANCE_CONSTANTS.STANDARD_DECELERATION * deltaTime_secs / 2;

  } else if (moment.speed < target.targetedSpeed) {
    speedChange = AIRCRAFT_PERFORMANCE_CONSTANTS.STANDARD_ACCELERATION * deltaTime_secs / 2;
  }

  moment.speed += speedChange;

  if (abs(speedChange) > abs(differenceBetweenPresentAndTargetSpeeds)) {
    moment.speed = target.targetedSpeed;
  }
}

void Aircraft::updateGroundSpeedPhysics(long deltaTime) {
  // This function is heavily based on the function of the same name in the AircraftModel class of the Openscope TRACON simulator project

  // Calculate true air speed vector
  const double indicatedAirspeed = moment.speed;
  const double trueAirspeed = indicatedAirspeed * (1 + (moment.altitude * AIRCRAFT_PHYSICS_CONSTANTS.TRUE_AIRSPEED_INCREASE_FACTOR_PER_FOOT));
  const vector<double> flightThroughAirVector = VectorMath::vecScale<double>(VectorMath::vectorizeHeading<double>(moment.heading), trueAirspeed);


  // Calculate ground speed and direction
  const vector<double> flightPathVector = flightThroughAirVector; //VectorMath::vecAdd(flightThroughAirVector, windVector);
  const double myGroundTrack = VectorMath::vec2Radial(flightPathVector);
  const double myGroundSpeed = VectorMath::vec2Len(flightPathVector);

  // cout << "myGroundSpeed: " << myGroundSpeed << endl;
  // Calculate new position

  const double hoursElapsed = UnitConverters::millisec_to_hour(deltaTime); //TimeKeeper.getDeltaTimeForGameStateAndTimewarp() * TIME.ONE_SECOND_IN_HOURS;
  const double distanceTraveled_nm = myGroundSpeed * hoursElapsed;

  // cout << "distanceTraveled: " << distanceTraveled_nm << endl;

  moment.currentPosition.setCoordinatesByBearingAndDistance(myGroundTrack, distanceTraveled_nm);

  moment.groundTrack = myGroundTrack;
  moment.groundSpeed = myGroundSpeed;
}

/*----------------------------------------
  This function is due to Greg James of Nvidia, via The Developer's Toolbox in the archives of Flipcode (http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml)
-----------------------------------------*/
bool Aircraft::pointIsInSeparationCylinder (const Position testPoint) {
  // vector d from line segment point 1 to point 2
  float dx, dy, dz;
  // vector pd from point 1 to test point
  float pdx, pdy, pdz;
  float dot, dsq;

  int altitudeDiff = moment.currentPosition.altitude - testPoint.altitude;
  if (abs(altitudeDiff) >= 1000) {
    return false;
  } else {

    float centerX = moment.currentPosition.longitude;
    float centerY = moment.currentPosition.latitude;
    float testX = testPoint.longitude;
    float testY = testPoint.latitude;
    float radius = float(AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_LATERAL_SEPARATION_NM) / float(GEO_CONSTANTS.EARTH_RADIUS_NM);


    float distX = pow(testX - centerX, 2) / float(GEO_CONSTANTS.EARTH_RADIUS_NM);
    float distY = pow(testY - centerY, 2) / float(GEO_CONSTANTS.EARTH_RADIUS_NM);
    // cout << "dist: " << pow(distX, 2) + pow(distY, 2) << endl;
    // cout << "radius^2: " << pow(radius, 2) << endl;
    if (distX + distY <= pow(radius, 2)) {
      return true;
    } else {
      return false;
    }

    // if (altitudeDiff < 0) {
      // dz = (moment.currentPosition.altitude + 1000) - (moment.currentPosition.altitude - 1000);
      // pdz = testPoint.altitude - moment.currentPosition.altitude;
    // } else {
      // dz = (moment.currentPosition.altitude - 1000) - (moment.currentPosition.altitude + 1000);
      // pdz = testPoint.altitude - moment.currentPosition.altitude;
    // }
  }



  // // length of the cylinder's vertical axis segment, squared
  // float length_sq = pow((AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_VERTICAL_SEPARATION_FT * 2), 2);
  // // radius of the bases of the cylinder, squared
  // float radius_sq = pow(float(AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_LATERAL_SEPARATION_NM) / float(GEO_CONSTANTS.EARTH_RADIUS_NM), 2);
  //
  //
  //
  // // trnslate so point one is origin; make vector from point 1 to point 2
  // // dx and dy are zero because the centers of the two end caps have the same latitude and longitude — they are separated only by altitude
  // dx = 0;
  // dy = 0;
  // //
  //
  //
  // // vector from point 1 to test point
  // pdx = testPoint.longitude - moment.currentPosition.longitude;
  // pdy = testPoint.latitude - moment.currentPosition.latitude;
  //
  // // Dot the d and pd vectors to see if point lies behind the cylinder cap centered on point 1
  // dot = (pdx * dx) + (pdy * dy) + (pdz * dz);
  //
  // cout << "Length sq: " << length_sq << endl;
  // cout << "Dot: " << dot << endl;
  // // If dot is less than zero, the point is behind the point 1 cap and thus not in the cylinder
  // // If greater than the cylinder axis line segment length squared, then the point is outside the other end cap at point 2
  // // if (dot < 0.0f || dot > length_sq) {
  //   // return false;
  // // } else {
  //   // Point lies within the parallel caps, so find distance squared from point to line
  //   // In short, where dist is pt distance to cyl axis:
    // 	// dist = sin( pd to d ) * |pd|
    // 	// distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // 	// dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // 	// dsq = pd * pd - dot * dot / lengthsq
    // 	//  where lengthsq is d*d or |d|^2 that is passed into this function
  //   dsq = ((pdx * pdx) + (pdy * pdy) + (pdz * pdz)) - ((dot * dot) / length_sq);
  //
  //   // cout << "dsq: " << dsq << endl;
  //   // cout << "radius^2: " << radius_sq << endl;
  //
  //   dsq /= GEO_CONSTANTS.EARTH_RADIUS_NM;
  //
  //   if (dsq > radius_sq) {
  //     return false;
  //   } else {
  //     return true;
  //   }
  // }
}

/*----------------------------------------
  This function is due to Greg James of Nvidia, via The Developer's Toolbox in the archives of Flipcode (http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml)
-----------------------------------------*/
bool Aircraft::pointIsInSeparationCylinderWithMyPosition (const Position &testPoint, const Position &myPosition) {

  int altitudeDiff = myPosition.altitude - testPoint.altitude;
  // cout << "Altitude Diff: " << altitudeDiff << endl;
  if (abs(altitudeDiff) >= 1000) {
    return false;
  } else {

    float centerX = myPosition.longitude;
    float centerY = myPosition.latitude;
    float testX = testPoint.longitude;
    float testY = testPoint.latitude;
    float radius = float(AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_LATERAL_SEPARATION_NM) / float(GEO_CONSTANTS.EARTH_RADIUS_NM);


    float distX = pow(testX - centerX, 2) / float(GEO_CONSTANTS.EARTH_RADIUS_NM);
    float distY = pow(testY - centerY, 2) / float(GEO_CONSTANTS.EARTH_RADIUS_NM);
    // cout << "dist: " << distX + distY << endl;
    // cout << "radius^2: " << pow(radius, 2) << endl;
    if (distX + distY <= pow(radius, 2)) {
      return true;
    } else {
      return false;
    }
  }

  // // length of the cylinder's vertical axis segment, squared
  // float length_sq = pow((AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_VERTICAL_SEPARATION_FT * 2), 2);
  // // radius of the bases of the cylinder, squared
  // float radius_sq = pow(float(AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_LATERAL_SEPARATION_NM) / float(GEO_CONSTANTS.EARTH_RADIUS_NM), 2);
  //
  // // vector d from line segment point 1 to point 2
  // float dx, dy, dz;
  // // vector pd from point 1 to test point
  // float pdx, pdy, pdz;
  // float dot, dsq;
  //
  // // trnslate so point one is origin; make vector from point 1 to point 2
  // // dx and dy are zero because the centers of the two end caps have the same latitude and longitude — they are separated only by altitude
  // dx = 0;
  // dy = 0;
  // //
  // dz = (myPosition.altitude + 1000) - (myPosition.altitude - 1000);
  //
  // // vector from point 1 to test point
  // pdx = testPoint.longitude - myPosition.longitude;
  // pdy = testPoint.latitude - myPosition.latitude;
  // pdz = testPoint.altitude - (myPosition.altitude);
  //
  // // Dot the d and pd vectors to see if point lies behind the cylinder cap centered on point 1
  // dot = (pdx * dx) + (pdy * dy) + (pdz * dz);
  //
  // // If dot is less than zero, the point is behind the point 1 cap and thus not in the cylinder
  // // If greater than the cylinder axis line segment length squared, then the point is outside the other end cap at point 2
  // if (dot < 0.0f || dot > length_sq) {
  //   return false;
  // } else {
  //   // Point lies within the parallel caps, so find distance squared from point to line
  //   // In short, where dist is pt distance to cyl axis:
    // 	// dist = sin( pd to d ) * |pd|
    // 	// distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // 	// dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // 	// dsq = pd * pd - dot * dot / lengthsq
    // 	//  where lengthsq is d*d or |d|^2 that is passed into this function
  //   dsq = ((pdx * pdx) + (pdy * pdy) + (pdz * pdz)) - ((dot * dot) / length_sq);
  //
  //   // cout << "dsq: " << dsq << endl;
  //   // cout << "radius^2: " << radius_sq << endl;
  //
  //   dsq /= GEO_CONSTANTS.EARTH_RADIUS_NM;
  //
  //   if (dsq > radius_sq) {
  //     return false;
  //   } else {
  //     return true;
  //   }
  // }
}

void Aircraft::updateOnMap() {
    updatePointStyle();
    updatePointGraphic();
    updateGraphicsOverlay();
}

void Aircraft::updatePointGraphic() {
    Esri::ArcGISRuntime::Point point = Esri::ArcGISRuntime::Point(moment.currentPosition.longitude, moment.currentPosition.latitude, Esri::ArcGISRuntime::SpatialReference::wgs84());
    graphic = new Esri::ArcGISRuntime::Graphic(point, parent);
}

void Aircraft::updatePointStyle() {
    float colorScaleFactor = (float(moment.altitude) / float(12000)) * 255;
    colorScaleFactor = min(colorScaleFactor, float(255));
    QColor color(255 - colorScaleFactor, (255 - colorScaleFactor) / 2, colorScaleFactor);


    Esri::ArcGISRuntime::SimpleMarkerSymbolStyle pointStyle;
    if (moment.hasConflict) {
        // create the point symbol
        pointStyle = Esri::ArcGISRuntime::SimpleMarkerSymbolStyle::Triangle;
    } else {
        pointStyle = Esri::ArcGISRuntime::SimpleMarkerSymbolStyle::Diamond;
    }
    Esri::ArcGISRuntime::SimpleMarkerSymbol * pointSymbol = new Esri::ArcGISRuntime::SimpleMarkerSymbol(pointStyle, color, 10), parent;
    graphicsOverlay->setRenderer(new Esri::ArcGISRuntime::SimpleRenderer(pointSymbol));
}

void Aircraft::updateGraphicsOverlay() {
    graphicsOverlay->graphics()->clear();
    graphicsOverlay->graphics()->append(graphic);
}

void Aircraft::removeFromMap() {
    graphicsOverlay->graphics()->clear();
}

void Aircraft::updatePosition() {

}

bool Aircraft::isAtOrigin() {
  if (moment.currentPosition.isEquivalentTo(*(path->origin->position))) {
    return true;
  } else {
    return false;
  }
}

bool Aircraft::isAtDestinationFix() {
  if (moment.currentPosition.isEquivalentTo(*(path->destination->position))) {
    return true;
  } else {
    return false;
  }
}

bool Aircraft::isAtNextFix() {
  if (moment.currentPosition.isEquivalentTo(*(moment.nextFix->position))) {
    return true;
  } else {
    return false;
  }
}

void Aircraft::draw (Position* spatialCenter, double scaleFactor, double panX, double panY) {

//  const float colorScaleFactor = float(moment.altitude) / float(12000);

//  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
//  glPushMatrix();
//  glColor3f(1 - colorScaleFactor, (1 - colorScaleFactor) / 2, colorScaleFactor);
//  glPointSize(6.0);
//  glTranslated((-spatialCenter->longitude + panX) * (scaleFactor), (-spatialCenter->latitude + panY) * (scaleFactor), 0);
//  glScaled(scaleFactor, scaleFactor, 1.0);
//  glBegin(GL_POINTS);
//    glVertex2d(moment.currentPosition.longitude, moment.currentPosition.latitude);
//  glEnd();

//  float radius = (AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_LATERAL_SEPARATION_NM + 1.5) / float(GEO_CONSTANTS.EARTH_RADIUS_NM);

//  drawSeparationRing();
//  glPopMatrix();
//  glPopAttrib();
}

/*----------------------------------------
  The code for drawing circles in OpenGL was adapted from Nicholas on Stack Overflow (https://stackoverflow.com/questions/28751268/create-circle-coordinates-given-central-lat-long-and-radius)
-----------------------------------------*/
void Aircraft::drawSeparationRing () {
//  glLineWidth(4.0);
//  glBegin(GL_LINE_LOOP);
//  double centerX = TrigMath::degreesToRadians(moment.currentPosition.longitude);
//  double centerY = TrigMath::degreesToRadians(moment.currentPosition.latitude);
//  double centerZ = TrigMath::degreesToRadians(moment.currentPosition.altitude);
//  for(int i = 0; i < 20; i++)
//  {
//      float radius = AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_LATERAL_SEPARATION_NM / float(GEO_CONSTANTS.EARTH_RADIUS_NM);

//      float theta = 2.0f * M_PI * float(i) / float(20);//get the current angle

//      float x = radius * (cosf(theta));//calculate the x component
//      float y = radius * (sinf(theta));//calculate the y component


//      double coordX = TrigMath::radiansToDegrees(x + centerX);
//      double coordY = TrigMath::radiansToDegrees(y + centerY);

//      glVertex3f(coordX, coordY, 0);//output vertex

//  }
//  glEnd();
}

void Aircraft::drawConflict(Position myPosition, Position otherAircraftPosition, string otherAircraftCallsign, Position* spatialCenter, double scaleFactor, double panX, double panY) {
  const int sqHeight = 0.018;

//  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  glPushMatrix();
//  // glColor3d(1.0, 0.0, 0.0);
//  glTranslated((-spatialCenter->longitude + panX) * (scaleFactor), (-spatialCenter->latitude + panY) * (scaleFactor), 0);
//  glScaled(scaleFactor, scaleFactor, 1.0);
//  glBegin(GL_POINTS);
//    if (otherAircraftCallsign == "DAL0") {
//      glColor3d(1.0, 0.0, 0.0);
//    } else if (otherAircraftCallsign == "UAL2") {
//      glColor3d(0.0, 1.0, 0.0);
//    } else if (otherAircraftCallsign == "AAL1") {
//      glColor3d(0.0, 0.0, 1.0);
//    } else {
//      glColor3d(1.0, 1.0, 0.0);
//    }
//    glVertex2d(myPosition.longitude, myPosition.latitude);

//    if (callsign == "DAL0") {
//      glColor3d(1.0, 0.0, 0.0);
//    } else if (callsign == "UAL2") {
//      glColor3d(0.0, 1.0, 0.0);
//    } else if (callsign == "AAL1") {
//      glColor3d(0.0, 0.0, 1.0);
//    } else {
//      glColor3d(1.0, 1.0, 0.0);
//    }
//    glVertex2d(otherAircraftPosition.longitude, otherAircraftPosition.latitude);
//    // // aircraft position
//    // // -- aircraft's colors
//    // glColor3dv(color);
//    // glVertex2d(myPosition.longitude - sqHeight, myPosition.latitude + sqHeight);
//    // glVertex2d(myPosition.longitude + sqHeight, myPosition.latitude + sqHeight);
//    // // -- otherAircraft's colors
//    // glColor3dv(otherAircraftColor);
//    // glVertex2d(myPosition.longitude + sqHeight, myPosition.latitude - sqHeight);
//    // glVertex2d(myPosition.longitude - sqHeight, myPosition.latitude - sqHeight);
//    //
//    // // aircraft position
//    // // -- aircraft's colors
//    // glColor3dv(color);
//    // glVertex2d(otherAircraftPosition.longitude - sqHeight, otherAircraftPosition.latitude + sqHeight);
//    // glVertex2d(otherAircraftPosition.longitude + sqHeight, otherAircraftPosition.latitude + sqHeight);
//    // // -- otherAircraft's colors
//    // glColor3dv(otherAircraftColor);
//    // glVertex2d(otherAircraftPosition.longitude + sqHeight, otherAircraftPosition.latitude - sqHeight);
//    // glVertex2d(otherAircraftPosition.longitude - sqHeight, otherAircraftPosition.latitude - sqHeight);


//    // // aircraft triangle
//    // glVertex2d(myPosition.longitude, myPosition.latitude + (triHeight / 2));
//    // glVertex2d(myPosition.longitude + (triHeight / 2), myPosition.latitude - (triHeight / 2));
//    // glVertex2d(myPosition.longitude - (triHeight / 2), myPosition.latitude - (triHeight / 2));
//    // // otherAircraft triangle
//    // glVertex2d(otherAircraftPosition.longitude, otherAircraftPosition.latitude + (triHeight / 2));
//    // glVertex2d(otherAircraftPosition.longitude + (triHeight / 2), otherAircraftPosition.latitude - (triHeight / 2));
//    // glVertex2d(otherAircraftPosition.longitude - (triHeight / 2), otherAircraftPosition.latitude - (triHeight / 2));

//  glEnd();
//  glPopMatrix();
//  glPopAttrib();

}
