#include "SpatialReference.h"
#include "Point.h"
#include "PointBuilder.h"
#include "Graphic.h"
#include "GraphicsOverlay.h"
#include "SimpleMarkerSymbol.h"

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>

#include "AircraftTarget.h"
#include "AircraftMoment.h"
#include "Position.h"
#include "FuturePosition.h"
#include "Path.h"
#include "Timer.h"

using namespace std;

#ifndef AIRCRAFT_H
#define AIRCRAFT_H

class Aircraft {
  public:
    // data members
    QObject * parent;
    Path* path;
    AircraftTarget target;
    AircraftMoment moment;
    std::map<long, FuturePosition*> futurePositions;
    float arrivalTime_zulu;
    long landingTime_ms;
    string callsign;
    bool isAtDestination;
    bool hasArrived;
    Esri::ArcGISRuntime::PointBuilder *pointBuilder;
    Esri::ArcGISRuntime::Graphic *graphic;
    Esri::ArcGISRuntime::GraphicsOverlay *graphicsOverlay;
    Esri::ArcGISRuntime::SimpleLineSymbolStyle lineStyle;

    // member functions
    Aircraft();
    Aircraft(QObject * myParent);
//    Aircraft(Aircraft &aircraftToCopy);
    void setPath(Path* newPath);
    void update (long deltaTime);
    void updateTargetedSpeed(int myTargetedSpeed = NULL);
    bool isAtOrigin();
    bool isAtDestinationFix();
    bool isAtNextFix();
    bool pointIsInSeparationCylinder (const Position testPoint);
    bool pointIsInSeparationCylinderWithMyPosition (const Position &testPoint, const Position &myPosition);
    // Position posAtTime (Time time);
    void draw(Position* spatialCenter, double scaleFactor, double panX, double panY);
    void drawConflict(Position myPosition, Position otherAircraftPosition, string otherAircraftCallsign, Position* spatialCenter, double scaleFactor, double panX, double panY);
    void generateFuturePositions(Timer timer, int step);
    vector<Position> willHaveConflictWithAircraft (Aircraft * otherAircraft, Timer timer, int futurePositionStep);
    void updateOnMap();
    void removeFromMap();


  private:
    void updateTarget();
    void updateTargetedHeading();
    void updateTargetedAltitude(int myTargetedAltitude = INT_MAX);
    void updateSpeedPhysics(long deltaTime);
    void updateGroundSpeedPhysics(long deltaTime);
    void updatePosition();
    void updateTurnPhysics (long deltaTime);
    void updateAltitudePhysics (long deltaTime);
    void decreaseAltitude (long deltaTime);
    void increaseAltitude (long deltaTime);
    double calculateTurnInitiaionDistance (Position nextFixPos);
    double calculateNewNominalCourse (Position myNextFixPos, Position myCurrentFixPos);
    double calculateCourseChangeInRadians (double currentHeading, double newNominalCourse);
    double calcTurnInitiationDistance (float speed, float bankAngle, double courseChange);
    double calcTurnRadius (float speed, float bankAngle);
    void drawSeparationRing();
    void updatePointGraphic();
    void updatePointStyle();
    void updateGraphicsOverlay();

};

#endif
