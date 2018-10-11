#include "PolylineBuilder.h"
#include "SpatialReference.h"
#include "Point.h"
#include "LineSegment.h"
#include "Part.h"
#include "PartCollection.h"
#include "Graphic.h"

#include <vector>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <climits>

#include "AirspaceGraph.h"
#include "Position.h"
#include "Fix.h"
#include "Path.h"

int MY_INFINITY = INT_MAX;

// TODO: I think this can be removed; Position has a default constructor now
AirspaceGraph::AirspaceGraph () {
  spatialAverage = new Position(0, 0);
  // easternmostFix = nullptr;
  // southernmostFix = nullptr;
  // westernmostFix = nullptr;
  // northernmostFix = nullptr;
}

AirspaceGraph::AirspaceGraph (vector<vector<int> > adjacencyMatrix) {
  adjMatrix = adjacencyMatrix;
  spatialAverage = new Position(0, 0);
  // easternmostFix = nullptr;
  // southernmostFix = nullptr;
  // westernmostFix = nullptr;
  // northernmostFix = nullptr;
}

void AirspaceGraph::addFix(string myName, double latitude, double longitude, string myType) {
  Fix newFix(myName, latitude, longitude, myType);
  fixes.push_back(newFix);
  recalcCenter(&newFix);
  recalcAverage(&newFix);
}

void AirspaceGraph::addFix(Fix fix) {
  cout << "addFix(" << fix.name << ")" << endl;
  cout << "address: " << &fix << endl;
  fixes.push_back(fix);
  recalcCenter(&fixes[fixes.size()-1]);
  recalcAverage(&fixes[fixes.size()-1]);
}

bool AirspaceGraph::recalcCenter (Fix *newFix) {
  cout << "Fixes.size(): " << fixes.size() << endl;


  // check if this fix is a new extreme
  // note: if fixes.size()==1, then this is the first fix, so it is by definition an extreme
  bool recalcCenter = false;
  if (fixes.size() == 1 || (newFix->position->latitude >= northernmostFix.position->latitude)) {
    northernmostFix = *newFix;
    recalcCenter = true;
  }
  if (fixes.size() == 1 || (newFix->position->latitude <= southernmostFix.position->latitude)) {
    southernmostFix = *newFix;
    recalcCenter = true;
  }
  if (fixes.size() == 1 || (newFix->position->longitude >= easternmostFix.position->longitude)) {
    easternmostFix = *newFix;
    recalcCenter = true;
  }
  if (fixes.size() == 1 || (newFix->position->longitude <= westernmostFix.position->longitude)) {
    westernmostFix = *newFix;
    recalcCenter = true;
  }
  if (recalcCenter) {
    double latd = (northernmostFix.position->latitude + southernmostFix.position->latitude) / double(2);
    double longd = (easternmostFix.position->longitude + westernmostFix.position->longitude) / double(2);
    spatialCenter = new Position(latd, longd);
  }

  return recalcCenter;
}

void AirspaceGraph::recalcAverage (Fix *newFix) {
    double newLatitude = ((spatialAverage->latitude * (fixes.size()-1)) + newFix->position->latitude) / fixes.size();
    double newLongitude = ((spatialAverage->longitude * (fixes.size()-1)) + newFix->position->longitude) / fixes.size();
    spatialAverage = new Position(newLatitude, newLongitude);
    // cout << "spatialAverage: " << spatialAverage->latitude << ", " << spatialAverage->longitude << endl;
}

Path* AirspaceGraph::dijkstra (int source, int destination) {
  vector<int> distance (adjMatrix[0].size());
  vector<int> unvisitedNodes;
  vector<int> previousNode (adjMatrix[0].size());

  for (int currentVert = 0; currentVert < adjMatrix[0].size(); currentVert++) {
    distance[currentVert] = MY_INFINITY;
    previousNode[currentVert] = NULL;
    unvisitedNodes.push_back(currentVert);
  }

  // cout << "Unvisited Nodes: ";
  // for (vector<int>::iterator it = unvisitedNodes.begin(); it != unvisitedNodes.end(); ++it) {
    // cout << *it;
  // }
  // cout << endl;

  distance[source] = 0;

  while (unvisitedNodes.size() > 0) {
    int u = findMinHelper(unvisitedNodes, distance);

    // cout << "unvisitedNodes: ";
    // for (int k = 0; k < unvisitedNodes.size(); k++) {
    //   cout << unvisitedNodes[k];
    // }

    vector<int>::iterator it = find(unvisitedNodes.begin(), unvisitedNodes.end(), u);
    // cout << "u: " << u << endl;
    unvisitedNodes.erase(it);

    cout.flush();

    for (int k = 0; k < unvisitedNodes.size(); k++) {
      int currentVert = unvisitedNodes[k];
      // if currentVert can be reached from u in the graph
      if (adjMatrix[u][currentVert] > 0) {
        // if the distance from source to currentVert via u is less than the currently recorded distance, set this as the new distance
        if ((distance[u] + adjMatrix[u][currentVert]) < distance[currentVert]) {
          distance[currentVert] = distance[u] + adjMatrix[u][currentVert];
          previousNode[currentVert] = u;
        }
      }
    }
  }

  return tracePathHelper(previousNode, source, destination);
}

Path* AirspaceGraph::dijkstraWithAdjMat (int source, int destination, std::vector<std::vector<int> > myAdjMat) {
  vector<int> distance (adjMatrix[0].size());
  vector<int> unvisitedNodes;
  vector<int> previousNode (adjMatrix[0].size());

  for (int currentVert = 0; currentVert < myAdjMat[0].size(); currentVert++) {
    distance[currentVert] = MY_INFINITY;
    previousNode[currentVert] = NULL;
    unvisitedNodes.push_back(currentVert);
  }

  // cout << "Unvisited Nodes: ";
  // for (vector<int>::iterator it = unvisitedNodes.begin(); it != unvisitedNodes.end(); ++it) {
    // cout << *it;
  // }
  // cout << endl;

  distance[source] = 0;

  while (unvisitedNodes.size() > 0) {
    int u = findMinHelper(unvisitedNodes, distance);

    // cout << "unvisitedNodes: ";
    // for (int k = 0; k < unvisitedNodes.size(); k++) {
    //   cout << unvisitedNodes[k];
    // }

    vector<int>::iterator it = find(unvisitedNodes.begin(), unvisitedNodes.end(), u);
    // cout << "u: " << u << endl;
    unvisitedNodes.erase(it);

    cout.flush();

    for (int k = 0; k < unvisitedNodes.size(); k++) {
      int currentVert = unvisitedNodes[k];
      // if currentVert can be reached from u in the graph
      if (myAdjMat[u][currentVert] > 0) {
        // if the distance from source to currentVert via u is less than the currently recorded distance, set this as the new distance
        if ((distance[u] + myAdjMat[u][currentVert]) < distance[currentVert]) {
          distance[currentVert] = distance[u] + myAdjMat[u][currentVert];
          previousNode[currentVert] = u;
        }
      }
    }
  }

  return tracePathHelper(previousNode, source, destination);
}

int AirspaceGraph::findMinHelper (vector<int> unvisitedNodes, vector<int> distance) {
  int min = MY_INFINITY;
  int minVert = NULL;

  for (int k = 0; k < unvisitedNodes.size(); k++) {
    int currentVert = unvisitedNodes[k];
    if (distance[currentVert] <= min) {
      min = distance[currentVert];
      minVert = currentVert;
    }
  }

  return minVert;
}

Path* AirspaceGraph::tracePathHelper (vector<int> previousNode, int source, int destination) {
  Path* path = new Path(&fixes[source], &fixes[destination]);

  // cout << "previous node:";
  for (int k = 0; k < previousNode.size(); k++) {
    cout << previousNode[k];
  }

  int nextNode = previousNode[destination];
  int k = 0;
  while (nextNode != source && k < 10) {
    path->fixSequence.insert(path->fixSequence.begin(), &fixes[nextNode]);
    nextNode = previousNode[nextNode];
  }

  // if there aren't any fixes in the fix sequence, no path was found between the origin and the destination
  // in this is the case, return the nullptr
  if (path->fixSequence.size() < 1) {
      return nullptr;
  }

  return path;
}
void AirspaceGraph::drawArrow (Fix * from, Fix * to) {
//  Fix * easternmoreFix = nullptr;
//  Fix * westernmoreFix = nullptr;
//  Fix * southernmoreFix = nullptr;
//  Fix * northernmoreFix = nullptr;
//  if (from->position->longitude > to->position->longitude) {
//    easternmoreFix = from;
//    westernmoreFix = to;
//  } else {
//    westernmoreFix = from;
//    easternmoreFix = to;
//  }
//  if (from->position->latitude > to->position->latitude) {
//    northernmoreFix = from;
//    southernmoreFix = to;
//  } else {
//    southernmoreFix = from;
//    northernmoreFix = to;
//  }

//  double a = easternmoreFix->position->longitude - westernmoreFix->position->longitude;
//  double b = easternmoreFix->position->latitude - westernmoreFix->position->latitude;
//  double c = sqrt(pow(a, 2) + pow(b, 2));
//  double cosine = b / c;
//  double theta = acos(cosine) * (180 / M_PI);

//  if (southernmoreFix == to) {
//    theta *= -1;
//  }
//  if (westernmoreFix == to) {
//    theta -= 180;
//  }
//  if (northernmoreFix == to && easternmoreFix == to) {
//    // theta -= 180;
//    theta *= -1;
//  }
//  if (westernmoreFix == to && northernmoreFix == to) {
//    theta *= -1;
//  }


//  double triangleWidth = 0.018;
//  double triangleHeight = 0.018;
//  Position * triangleCenter = new Position(to->position->latitude - (triangleHeight / 2), to->position->longitude);
//  Position * t1 = new Position(triangleCenter->latitude - (triangleHeight / 2), triangleCenter->longitude - (triangleHeight / 2));
//  Position * t2 = new Position(triangleCenter->latitude - (triangleHeight / 2), triangleCenter->longitude + (triangleHeight / 2));
//  Position * t3 = new Position(triangleCenter->latitude + (triangleHeight / 2), triangleCenter->longitude);

//  glPushAttrib(GL_CURRENT_BIT);
//  glPushMatrix();
//  glTranslated(to->position->longitude, to->position->latitude, 0);
//    glRotated(theta, 0, 0, 1.0);
//    glTranslated(-1 * to->position->longitude, -1 * to->position->latitude, 0);
//    glBegin(GL_TRIANGLES);
//      glColor3f(0.0, 0.0, 0.1);
//      glVertex2d(t1->longitude, t1->latitude);
//      glVertex2d(t2->longitude, t2->latitude);
//      glColor3f(1.0, 0.0, 0.0);
//      glVertex2f(t3->longitude, t3->latitude);
//    glEnd();
//  glPopMatrix();
//  glPopAttrib();
}

using namespace Esri::ArcGISRuntime;

Graphic* AirspaceGraph::generateGraphic () {
    PolylineBuilder* polylineBuilder = new PolylineBuilder(SpatialReference(4326), nullptr);
    PartCollection* partCollection = new PartCollection(SpatialReference(4326), nullptr);
    for (int i = 0; i < adjMatrix.size(); i++) {
        for (int j = 0; j < adjMatrix.size(); j++) {
            if (adjMatrix[i][j] > 0) {
                LineSegment segmentToAdd(fixes[i].position->longitude, fixes[i].position->latitude, fixes[j].position->longitude, fixes[j].position->latitude, SpatialReference::wgs84());
                Part *partToAdd = new Part(SpatialReference(4326));
                partToAdd->addSegment(segmentToAdd);
                partCollection->addPart(partToAdd);
            }
        }
    }

    polylineBuilder->setParts(partCollection);
    Graphic * graphicToReturn = new Graphic(polylineBuilder->toGeometry());

    return graphicToReturn;

}

void AirspaceGraph::drawGraph (double scaleFactor, double panX, double panY) {
//  glPushMatrix();
//  glTranslated((-spatialCenter->longitude + panX) * (scaleFactor), (-spatialCenter->latitude + panY) * (scaleFactor), 0);
//  glScaled(scaleFactor, scaleFactor, 1.0);
//  // glScaled(scaleFactor, scaleFactor, 1.0);
//  drawGraphFixes();
//  drawGraphLines();
//  glPopMatrix();
}

void AirspaceGraph::drawGraphFixes () {
//  glBegin(GL_POINTS);
//  for (int i = 0; i < fixes.size(); i++) {
//    glVertex2d(fixes[i].position->longitude, fixes[i].position->latitude);
//  }
//  glEnd();
}

void AirspaceGraph::drawGraphLines () {
//  glLineWidth(1.0);
//  for (int i = 0; i < adjMatrix.size(); i++) {
//    for (int j = 0; j < adjMatrix[i].size(); j++) {
//      if (adjMatrix[i][j] > 0) {
//        // drawArrow(&fixes[i], &fixes[j]);
//        glBegin(GL_LINES);
//          glVertex2d(fixes[i].position->longitude, fixes[i].position->latitude);
//          glVertex2d(fixes[j].position->longitude, fixes[j].position->latitude);
//        glEnd();
//      }
//    }
//  }
}

void AirspaceGraph::drawShortestPath (int source, int destination, double scaleFactor, double panX, double panY) {
//  Path* path = dijkstra(source, destination);


//  vector<Fix*> pathCopy = path->fixSequence;
//  pathCopy.insert(pathCopy.begin(), &fixes[source]);
//  pathCopy.push_back(&fixes[destination]);

//  cout << "Path: ";
//  for (int k = 0; k < pathCopy.size(); k++) {
//    cout << pathCopy[k]->name << ", ";
//  }
//  cout << endl;

//  // cout << "path size: " << path.size();
//  glPushMatrix();
//  glPushAttrib(GL_CURRENT_BIT);
//    glColor3f(0.0, 0.0, 1.0);
//    glTranslated((-spatialCenter->longitude + panX) * (scaleFactor), (-spatialCenter->latitude + panY) * (scaleFactor), 0);
//    glScaled(scaleFactor, scaleFactor, 1.0);
//    glLineWidth(4.0);
//    for (int k = 0; k < (pathCopy.size() - 1); k++) {
//      glBegin(GL_LINES);
//        glVertex2d(pathCopy[k]->position->longitude, pathCopy[k]->position->latitude);
//        glVertex2d(pathCopy[k+1]->position->longitude, pathCopy[k+1]->position->latitude);
//      glEnd();
//    }
//  glPopAttrib();
//  glPopMatrix();
}
