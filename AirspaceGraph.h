#include <Graphic.h>

#include <stdio.h>
#include <vector>

#include "Fix.h"
#include "Position.h"
#include "Path.h"

using namespace std;

#ifndef AIRSPACE_GRAPH_H
#define AIRSPACE_GRAPH_H

class AirspaceGraph {
  public:
    AirspaceGraph ();
    AirspaceGraph (vector<vector<int> > adjacencyMatrix);
    vector<vector<int > > adjMatrix;
    vector<Fix> fixes;
    Position *spatialAverage;
    Position *spatialCenter;
    Fix northernmostFix, easternmostFix, southernmostFix, westernmostFix;

    Path* dijkstra (int source, int destination);
    Path* dijkstraWithAdjMat (int source, int destination, std::vector<std::vector<int> > myAdjMat);
    void addFix(string myName, double latitude, double longitude, string myType);
    void addFix(Fix fix);
    void drawGraph (double scaleFactor, double panX, double panY);
    Esri::ArcGISRuntime::Graphic* generateGraphic();
    void drawShortestPath(int source, int destination, double scaleFactor, double panX, double panY);


  private:
    int findMinHelper (vector<int> unvisitedNodes, vector<int> distance);
    Path* tracePathHelper (vector<int> previousNode, int source, int destination);
    void drawArrow (Fix * from, Fix * to);
    void drawGraphFixes ();
    void drawGraphLines ();
    bool recalcCenter (Fix *newFix);
    void recalcAverage (Fix *newFix);

};

#endif
