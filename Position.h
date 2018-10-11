#include <stdlib.h>
#include <vector>

#ifndef POSITION_H
#define POSITION_H

using namespace std;

class Position {
  public:
    // data members
    double latitude;
    double longitude;
    int altitude;

    // member functions
    Position();
    Position(const Position &pos);
    Position(double myLatitude, double myLongitude);
    Position(double myLatitude, double myLongitude, int myAltitude);
    vector<double> getxyVec ();
    vector<double> getLatLongVec();
    double distanceTo(Position toPos);
    double bearingTo(Position pos);
    void setCoordinatesLatLon(double lat, double lon);
    void setCoordinatesByBearingAndDistance(double bearing, double distance);
    bool isEquivalentTo(Position pos);
};

#endif
