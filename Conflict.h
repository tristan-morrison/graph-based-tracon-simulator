#ifndef CONFLICT_H
#define CONFLICT_H

#include <unordered_map>
#include <tuple>
#include <utility>

#include "GraphBasedTraconSimulator.h"
#include "Aircraft.h"
#include "AirspaceGraph.h"
#include "FuturePosition.h"
#include "Path.h"

class Conflict
{
public:
    Aircraft *aircraft1;
    Aircraft *aircraft2;
    std::unordered_map<Aircraft*, Aircraft> aircraftCopies;
    long startTime_msOffset;
    AirspaceGraph* airspaceGraph;
    std::vector<std::vector<int > > myAdjMatrix;
    std::tuple<int, int, int> edgeRemoved;

    Conflict(Aircraft *myAircraft1, Aircraft *myAircraft2, long myStartTime, AirspaceGraph* myAirspaceGraph);
    std::pair<Aircraft*, Path> generatePathResolution(GraphBasedTraconSimulator * parentSimulator);
    static Conflict* detectConflictsBetweenAircraft(Aircraft * aircraft1, Aircraft * aircraft2, Timer timer, int futurePositionStep, AirspaceGraph* airspaceGraph);

    std::pair<Aircraft*, float> generateSpeedResolution(GraphBasedTraconSimulator * parentSimulator);

private:
    void removeEdgeFromAdjMatrix(int fromFixIndex, int toFixIndex);
};

#endif // CONFLICT_H
