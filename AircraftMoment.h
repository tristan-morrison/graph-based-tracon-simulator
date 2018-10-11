#include <stdlib.h>

#include "Position.h"
#include "Fix.h"

#ifndef AIRCRAFT_MOMENT_H
#define AIRCRAFT_MOMENT_H

class AircraftMoment {
  public:
    Position currentPosition;
    Fix *nextFix, *previousFix;
    int nextFix_index;
    int previousFix_index;
    float arcLengthCoord;
    double heading;
    float altitude;
    int altitudeTrend;
    float speed = 250;
    float groundSpeed;
    float groundTrack;
    bool isAtDestination;
    bool hasConflict;

    AircraftMoment();
};

#endif
