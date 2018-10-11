#include <stdlib.h>

#ifndef AIRCRAFT_TARGET_H
#define AIRCRAFT_TARGET_H

class AircraftTarget {
  public:
    double targetedHeading;
    float targetedAltitude;
    int targetedSpeed;

    AircraftTarget ();

};

#endif
