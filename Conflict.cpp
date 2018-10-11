#include <unordered_map>
#include <utility>
#include <tuple>

#include "Conflict.h"
#include "Aircraft.h"
#include "AirspaceGraph.h"
#include "FuturePosition.h"
#include "Path.h"
#include "constants.h"


Conflict::Conflict(Aircraft *myAircraft1, Aircraft *myAircraft2, long myStartTime, AirspaceGraph* myAirspaceGraph) {

    aircraft1 = myAircraft1;
    aircraft2 = myAircraft2;

    std::unordered_map<Aircraft*, Aircraft> myAircraftCopies;
    Aircraft aircraft1Copy(*myAircraft1);
    myAircraftCopies.emplace(aircraft1, aircraft1Copy);
    Aircraft aircraft2Copy(*myAircraft2);
    myAircraftCopies.emplace(aircraft2, aircraft2Copy);
    aircraftCopies = myAircraftCopies;

    airspaceGraph = myAirspaceGraph;
    myAdjMatrix = airspaceGraph->adjMatrix;

    std::tuple<int, int, int> myEdgeRemoved;
    std::get<0>(myEdgeRemoved) = INT_MAX;
    std::get<1>(myEdgeRemoved) = INT_MAX;
    std::get<2>(myEdgeRemoved) = INT_MAX;
    edgeRemoved = myEdgeRemoved;

    startTime_msOffset = myStartTime;

}

Conflict* Conflict::detectConflictsBetweenAircraft(Aircraft * aircraft1, Aircraft * aircraft2, Timer timer, int futurePositionStep, AirspaceGraph* airspaceGraph) {
  long aircraft1LandingTime = aircraft1->landingTime_ms;
  long aircraft2LandingTime = aircraft2->landingTime_ms;
  long lastPotentialConflictTime = min(aircraft1LandingTime, aircraft2LandingTime);


  long aircraft1ArrivalTime_ms = timer.getMillisecondsOffsetFromStart(aircraft1->arrivalTime_zulu) + timer.startTime_ms;
  long aircraft1FirstRecordedTime = aircraft1->futurePositions.lower_bound(aircraft1ArrivalTime_ms)->first;
  long aircraft2ArrivalTime_ms = timer.getMillisecondsOffsetFromStart(aircraft2->arrivalTime_zulu) + timer.startTime_ms;
  long aircraft2FirstRecordedTime = aircraft2->futurePositions.lower_bound(aircraft2ArrivalTime_ms)->first;
  long firstPotentialConflictTime = max(aircraft1FirstRecordedTime, aircraft2FirstRecordedTime);

  cout << "first pot. conflict time: " << firstPotentialConflictTime << endl;
  cout << "last pot. conflict time: " << lastPotentialConflictTime << endl;

  cout << "Checking for conflict between " << aircraft1->callsign << " and " << aircraft2->callsign <<  endl;

  // for (map<long, Position>::iterator it = aircraft2->futurePositions.begin(); it!=aircraft2->futurePositions.end(); ++it) {
    // cout << it->first << ": " << (it->second).latitude << endl;
  // }
  // cout << "finished printing aircraft2->futurePositions" << endl;


  for (long potentialConflictTime = firstPotentialConflictTime; potentialConflictTime <= lastPotentialConflictTime; potentialConflictTime += futurePositionStep) {
    Position aircraft1PosAtTime = aircraft1->futurePositions[potentialConflictTime]->position;
    Position aircraft2PosAtTime = aircraft2->futurePositions[potentialConflictTime]->position;
    cout << aircraft1->callsign << " @ " << potentialConflictTime << ": " << aircraft1PosAtTime.latitude << ", " << aircraft1PosAtTime.longitude << endl;
    cout << aircraft2->callsign << " @ " << potentialConflictTime << ": " << aircraft2PosAtTime.latitude << ", " << aircraft2PosAtTime.longitude << endl;

    // cout << "Record time: " << potentialConflictTime << endl;
    if (aircraft1->pointIsInSeparationCylinderWithMyPosition(aircraft2PosAtTime, aircraft1PosAtTime)) {
      cout << "Time of conflict between " << aircraft1->callsign << " and " << aircraft2->callsign <<  ": " << potentialConflictTime << endl;
      Conflict * retConflict = new Conflict(aircraft1, aircraft2, potentialConflictTime, airspaceGraph);

      return retConflict;
    }
  }

  cout << "No conflict found; returning nullptr" << endl;
  return nullptr;

}

std::pair<Aircraft*, Path> Conflict::generatePathResolution(GraphBasedTraconSimulator * parentSimulator) {

    Aircraft *aircraftToReroute, *otherAircraft;

    // find out which aircraft is closer to its destination
    // we will be speeding up the one that is closer and slowing down the one that is further
    float aircraft1DistanceToDest = aircraft1->path->distanceToDestination_nm(&aircraftCopies[aircraft1].futurePositions[startTime_msOffset]->position, aircraftCopies[aircraft1].moment.nextFix_index);
    float aircraft2DistanceToDest = aircraft2->path->distanceToDestination_nm(&aircraftCopies[aircraft2].futurePositions[startTime_msOffset]->position, aircraftCopies[aircraft2].moment.nextFix_index);
    if (aircraft1DistanceToDest > aircraft2DistanceToDest) {
        aircraftToReroute = aircraft1;
        otherAircraft = aircraft2;
    } else {
        aircraftToReroute = aircraft2;
        otherAircraft = aircraft1;
    }

    // determine which edge of the graph this aircraft is currently on and remove that from the adjacency matrix
    int toFixIndexInSequence = aircraftCopies[aircraftToReroute].futurePositions[startTime_msOffset]->nextFixIndex;
    int prevFixIndexInSequence = aircraftCopies[aircraftToReroute].futurePositions[startTime_msOffset]->prevFixIndex;

//    std::vector<std::pair<int, int> > edgesToRemove;
//    edgesToRemove.push_back(std::pair<int, int>(prevFixIndexInSequence, toFixIndexInSequence));

    std::vector<std::pair<int, int> > edgesToRemove;
    if (aircraftCopies[aircraftToReroute].path->fixSequence.size() > 0) {
        // if there is one or more fix in fixSequence
        edgesToRemove.push_back(std::pair<int, int>(aircraftCopies[aircraftToReroute].path->origin->indexInGraph, aircraftCopies[aircraftToReroute].path->fixSequence[0]->indexInGraph));
        for (int k = 0; k < aircraftCopies[aircraftToReroute].path->fixSequence.size()-1; k++) {
            edgesToRemove.push_back(std::pair<int, int>(aircraftCopies[aircraftToReroute].path->fixSequence[k]->indexInGraph, aircraftCopies[aircraftToReroute].path->fixSequence[k+1]->indexInGraph));
        }
        int indexOfLast = aircraftCopies[aircraftToReroute].path->fixSequence.size() - 1;
        edgesToRemove.push_back(std::pair<int, int>(aircraftCopies[aircraftToReroute].path->fixSequence[indexOfLast]->indexInGraph, aircraftCopies[aircraftToReroute].path->destination->indexInGraph));
    } else {
        // if there are no fixes in fixSequence, then the only edge in the path is that between origin and destination
        edgesToRemove.push_back(std::pair<int, int>(aircraftCopies[aircraftToReroute].path->origin->indexInGraph, aircraftCopies[aircraftToReroute].path->destination->indexInGraph));
    }

    for (int k = 0; k < edgesToRemove.size(); k++) {
        removeEdgeFromAdjMatrix(edgesToRemove[k].first, edgesToRemove[k].second);

        Path * newPath = airspaceGraph->dijkstraWithAdjMat(aircraftToReroute->path->origin->indexInGraph, aircraftToReroute->path->destination->indexInGraph, myAdjMatrix);

        if (newPath != nullptr) {
            aircraftCopies[aircraftToReroute].setPath(newPath);
            aircraftCopies[aircraftToReroute].generateFuturePositions(parentSimulator->timer, parentSimulator->futurePositionStep);
            Conflict * conflict = detectConflictsBetweenAircraft(&aircraftCopies[aircraftToReroute], &aircraftCopies[otherAircraft], parentSimulator->timer, parentSimulator->futurePositionStep, &(parentSimulator->airspaceGraph));
            if (conflict == nullptr) {
                return std::pair<Aircraft*, Path>(aircraftToReroute, *newPath);
            }
        }
    }

    qDebug() << "NO PATH SOLUTION FOUND" << endl;
}

void Conflict::removeEdgeFromAdjMatrix(int fromFixIndex, int toFixIndex) {

    // put the edge currently removed back in the graph
    if (std::get<0>(edgeRemoved) < INT_MAX) {
        myAdjMatrix[std::get<0>(edgeRemoved)][std::get<1>(edgeRemoved)] = std::get<2>(edgeRemoved);
    }


    // remove next edge from adj matrix
    std::get<0>(edgeRemoved) = fromFixIndex;
    std::get<1>(edgeRemoved) = toFixIndex;
    std::get<2>(edgeRemoved) = myAdjMatrix[fromFixIndex][toFixIndex];

    myAdjMatrix[fromFixIndex][toFixIndex] = 0;
}

std::pair<Aircraft*, float> Conflict::generateSpeedResolution (GraphBasedTraconSimulator* parentSimulator) {

    Aircraft *aircraftToSlow, *aircraftToSpeedUp;

    // find out which aircraft is closer to its destination
    // we will be speeding up the one that is closer and slowing down the one that is further
    float aircraft1DistanceToDest = aircraft1->path->distanceToDestination_nm(&aircraftCopies[aircraft1].futurePositions[startTime_msOffset]->position, aircraftCopies[aircraft1].moment.nextFix_index);
    float aircraft2DistanceToDest = aircraft2->path->distanceToDestination_nm(&aircraftCopies[aircraft2].futurePositions[startTime_msOffset]->position, aircraftCopies[aircraft2].moment.nextFix_index);
    if (aircraft1DistanceToDest > aircraft2DistanceToDest) {
        aircraftToSlow = aircraft1;
        aircraftToSpeedUp = aircraft2;
    } else {
        aircraftToSlow = aircraft2;
        aircraftToSpeedUp = aircraft1;
    }

    float nextTargetedSpeed = aircraftToSlow->target.targetedSpeed - 10;
    while (aircraftCopies[aircraftToSlow].target.targetedSpeed  > AIRCRAFT_PERFORMANCE_CONSTANTS.MINIMUM_SPEED_KN) {
        aircraftCopies[aircraftToSlow].updateTargetedSpeed(nextTargetedSpeed);
        aircraftCopies[aircraftToSlow].generateFuturePositions(parentSimulator->timer, parentSimulator->futurePositionStep);
        Conflict * conflict = detectConflictsBetweenAircraft(&aircraftCopies[aircraftToSlow], &aircraftCopies[aircraftToSpeedUp], parentSimulator->timer, parentSimulator->futurePositionStep, &(parentSimulator->airspaceGraph));
        if (conflict == nullptr) {
            return std::pair<Aircraft*, float>(aircraftToSlow, nextTargetedSpeed);
        }
        nextTargetedSpeed -= 10;
    }

}
