// Copyright 2016 ESRI
//
// All rights reserved under the copyright laws of the United States
// and applicable international laws, treaties, and conventions.
//
// You may freely redistribute and use this sample code, with or
// without modification, provided you include the original copyright
// notice and use restrictions.
//
// See the Sample code usage restrictions document for further information.
//

#include <QTimer>
#include <QElapsedTimer>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QString>

#include "Map.h"
#include "MapQuickView.h"
#include "SpatialReference.h"
#include "Point.h"
#include "PolylineBuilder.h"
#include "Polyline.h"
#include "PolygonBuilder.h"
#include "Polygon.h"
#include "Graphic.h"
#include "GraphicsOverlay.h"
#include "SimpleMarkerSymbol.h"
#include "SimpleLineSymbol.h"
#include "SimpleFillSymbol.h"
#include "SimpleRenderer.h"

#include "GraphBasedTraconSimulator.h"

#include <vector>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <math.h>
#include <map>
#include <climits>

#include "AirspaceGraph.h"
#include "Fix.h"
#include "CSVRow.hpp"
#include "CSVRow_QFile.h"
#include "Path.h"
#include "Aircraft.h"
#include "utilities.h"
#include "constants.h"
#include "VectorMath.h"
#include "Timer.h"
#include "Conflict.h"
//#include "Simulator.h"

using namespace Esri::ArcGISRuntime;

GraphBasedTraconSimulator::GraphBasedTraconSimulator(QQuickItem* parent /* = nullptr */):
    QQuickItem(parent)
{
    futurePositionStep = 100;
//    // create the point symbol
//    SimpleMarkerSymbol* pointSymbol = new SimpleMarkerSymbol(SimpleMarkerSymbolStyle::Diamond, QColor("red"), 10, this);

//    aircraftGraphicsOverlay = new GraphicsOverlay(this);
    // set the renderer of the overlay to be the marker symbol
//    aircraftGraphicsOverlay->setRenderer(new SimpleRenderer(pointSymbol, this));
    // add the overlay to the mapview
//    m_mapView->graphicsOverlays()->append(aircraftGraphicsOverlay);
}

GraphBasedTraconSimulator::~GraphBasedTraconSimulator()
{
}

void GraphBasedTraconSimulator::componentComplete()
{
    QQuickItem::componentComplete();

    // find QML MapView component
    m_mapView = findChild<MapQuickView*>("mapView");

    // Create a map using the topographic Basemap
    m_map = new Map(BasemapType::Topographic, 33.867886, -118, 10, this);

    // Set map to map view
    m_mapView->setMap(m_map);

//    addGraphicsOverlay();
    loadAirspaceGraph();

    connect(&loopTimer, SIGNAL(timeout()), this, SLOT(iterateMainLoop()));
    elapsedTimer.start();
    loopTimer.start(1000);


}

void GraphBasedTraconSimulator::iterateMainLoop() {
    qDebug() << "Timer startTime_ms: " << timer.startTime_ms << endl;

    long deltaTime = elapsedTimer.restart();
    timer.recordTick(deltaTime);

    if (aircraft[0]->hasArrived) {
        aircraft[0]->update(timer.getSimulatedDeltaTime());
//        aircraftGraphicsOverlay->graphics()->append(aircraft[0]->graphic);
    } else if (timer.zuluTimeHasCome(aircraft[0]->arrivalTime_zulu)) {
        aircraft[0]->hasArrived = true;
        aircraft[0]->update(timer.getSimulatedDeltaTime());
    }
    if (aircraft[1]->hasArrived) {
        aircraft[1]->update(timer.getSimulatedDeltaTime());
//        aircraftGraphicsOverlay->graphics()->append(aircraft[1]->graphic);
    } else if (timer.zuluTimeHasCome(aircraft[1]->arrivalTime_zulu)) {
        aircraft[1]->hasArrived = true;
        aircraft[1]->update(timer.getSimulatedDeltaTime());
    }
    if (aircraft[2]->hasArrived) {
        aircraft[2]->update(timer.getSimulatedDeltaTime());
//        aircraftGraphicsOverlay->graphics()->append(aircraft[2]->graphic);
    } else if (timer.zuluTimeHasCome(aircraft[2]->arrivalTime_zulu)) {
        aircraft[2]->hasArrived = true;
        aircraft[2]->update(timer.getSimulatedDeltaTime());
    }

    aircraft[0]->moment.hasConflict = false;
    aircraft[1]->moment.hasConflict = false;
    aircraft[2]->moment.hasConflict = false;
    if (aircraft[0]->pointIsInSeparationCylinder(aircraft[2]->moment.currentPosition)) {
        aircraft[0]->moment.hasConflict = true;
        aircraft[2]->moment.hasConflict = true;
    }
    if (aircraft[0]->pointIsInSeparationCylinder(aircraft[1]->moment.currentPosition)) {
        aircraft[0]->moment.hasConflict = true;
        aircraft[1]->moment.hasConflict = true;
    }
    if (aircraft[2]->pointIsInSeparationCylinder(aircraft[1]->moment.currentPosition)) {
        qDebug() << "AIRCRAFT 1 HAS A CONFLICT WITH AIRCRAFT 2" << endl;
        aircraft[2]->moment.hasConflict = true;
        aircraft[1]->moment.hasConflict = true;
    }

    if (aircraft[0]->hasArrived && !aircraft[0]->moment.isAtDestination) {
        aircraft[0]->updateOnMap();
    }
    if (aircraft[1]->hasArrived && !aircraft[1]->moment.isAtDestination) {
        aircraft[1]->updateOnMap();
    }
    if (aircraft[2]->hasArrived && !aircraft[2]->moment.isAtDestination) {
        aircraft[2]->updateOnMap();
    }


    if (!aircraft[0]->moment.isAtDestination || !aircraft[1]->moment.isAtDestination || !aircraft[2]->moment.isAtDestination) {
        loopTimer.start(1000);
    } else {
        loopTimer.stop();
    }
}

void GraphBasedTraconSimulator::loadAirspaceGraph() {
    qDebug() << "I made it to the loadAirspaceGraph function!" << endl;

    map<string, Fix> allFixes;
    QFile allFixesFile(":/zla/Resources/ZLA.csv");
    allFixesFile.open(QIODevice::ReadOnly);
    QTextStream allFixesFileStream(&allFixesFile);
//    ifstream allFixesFile(":/zla/ZLA.csv");
    CSVRowQFile row;
    QString dummyString;
    // read the first line, the line of column headers, into a dummy string
    allFixesFileStream.readLineInto(&dummyString);
    while (!allFixesFileStream.atEnd()) {
        allFixesFileStream >> row;
        qDebug() << row[0] << endl;
        Fix newFix(row[0].toStdString(), row[1].toDouble(), row[2].toDouble(), row[5].toStdString());
        allFixes.insert(pair<string, Fix>(row[0].toStdString(), newFix));
    }

    QFile graphAltitudesFile(":zla/Resources/ZLAGraph_Altitudes.csv");
    graphAltitudesFile.open(QIODevice::ReadOnly);
    QTextStream graphAltitudesFileStream(&graphAltitudesFile);
    while (!graphAltitudesFileStream.atEnd()) {
        graphAltitudesFileStream >> row;
        allFixes[row[0].toStdString()].altitude = row[1].toInt();
        allFixes[row[0].toStdString()].speed = row[2].toInt();
        if (allFixes[row[0].toStdString()].speed > 9000) {
            // in the config files, a speed of 9999 is used to indicate final approach speed
            // in the software, we use INT_MAX to indicate this
            allFixes[row[0].toStdString()].speed = INT_MAX;
        }
    }

    vector<vector<string> > pairs;
    QFile graphFile(":/zla/Resources/ZLAGraph.csv");
    graphFile.open(QIODevice::ReadOnly);
    QTextStream graphFileStream(&graphFile);
    int k = 0;
    while (!graphFileStream.atEnd()) {
        graphFileStream >> row;
        vector<string> newPair;
        newPair.push_back(row[0].toStdString());
        newPair.push_back(row[1].toStdString());
        pairs.push_back(newPair);

        if (allFixes[row[0].toStdString()].indexInGraph >= INT_MAX) {
            airspaceGraph.addFix(allFixes[row[0].toStdString()]);
            allFixes[row[0].toStdString()].indexInGraph = k;
            k++;
        }
        if (allFixes[row[1].toStdString()].indexInGraph >= INT_MAX) {
            airspaceGraph.addFix(allFixes[row[1].toStdString()]);
            allFixes[row[1].toStdString()].indexInGraph = k;
            k++;
        }
    }

    vector<vector<int> > myAdjacencyMatrix(k, vector<int>(k, 0));
    for (int i = 0; i < pairs.size(); i++) {
        Fix fromFix = allFixes[pairs[i][0]];
        Fix toFix = allFixes[pairs[i][1]];
        double weight = fromFix.position->distanceTo(*(toFix.position));
        myAdjacencyMatrix[fromFix.indexInGraph][toFix.indexInGraph] = weight * 100;
    }

    airspaceGraph.adjMatrix = myAdjacencyMatrix;

    SimpleLineSymbol* lineSymbol = new SimpleLineSymbol(SimpleLineSymbolStyle::Solid, QColor("blue"), 2, this);

    airspaceGraphGraphicsOverlay = new GraphicsOverlay(this);
    airspaceGraphGraphicsOverlay->setRenderer(new SimpleRenderer(lineSymbol, this));
    Graphic * airspaceGraphGraphic = airspaceGraph.generateGraphic();
    airspaceGraphGraphicsOverlay->graphics()->append(airspaceGraphGraphic);
    m_mapView->graphicsOverlays()->append(airspaceGraphGraphicsOverlay);

    SimpleLineSymbol* pathsLineSymbol = new SimpleLineSymbol(SimpleLineSymbolStyle::Solid, QColor("green"), 5, this);
    pathsGraphicsOverlay = new GraphicsOverlay(this);
    pathsGraphicsOverlay->setRenderer(new SimpleRenderer(pathsLineSymbol, this));
    m_mapView->graphicsOverlays()->append(pathsGraphicsOverlay);

    vector<Aircraft*> vec(3, nullptr);
    aircraft = vec;
    aircraft[0] = new Aircraft(this);
    aircraft[1] = new Aircraft(this);
    aircraft[2] = new Aircraft(this);

    aircraft[0]->moment.currentPosition = *(airspaceGraph.fixes[0].position);
    aircraft[0]->setPath(airspaceGraph.dijkstra(0, 30));
    aircraft[0]->moment.altitude = aircraft[0]->path->origin->altitude;
    aircraft[0]->moment.speed = aircraft[0]->path->origin->speed;
    aircraft[0]->arrivalTime_zulu = 1200;
    aircraft[0]->callsign = "DAL0";
//    aircraft[0]->graphic = new Graphic(Point(aircraft[0]->moment.currentPosition.longitude, aircraft[0]->moment.currentPosition.latitude, SpatialReference::wgs84()));
    m_mapView->graphicsOverlays()->append(aircraft[0]->graphicsOverlay);
    pathsGraphicsOverlay->graphics()->append(aircraft[0]->path->generateGraphic(this));

    aircraft[1]->moment.currentPosition = *(airspaceGraph.fixes[0].position);
    aircraft[1]->setPath(airspaceGraph.dijkstra(0, 12));
    aircraft[1]->moment.altitude = aircraft[1]->path->origin->altitude;
    aircraft[1]->moment.speed = aircraft[1]->path->origin->speed;
    aircraft[1]->arrivalTime_zulu = 1205;
    aircraft[1]->callsign = "AAL1";
//    aircraft[1]->graphic = new Graphic(Point(aircraft[1]->moment.currentPosition.longitude, aircraft[0]->moment.currentPosition.latitude, SpatialReference::wgs84()));
    m_mapView->graphicsOverlays()->append(aircraft[1]->graphicsOverlay);

    aircraft[2]->moment.currentPosition = *(airspaceGraph.fixes[46].position);
    aircraft[2]->setPath(airspaceGraph.dijkstra(46, 30));
    aircraft[2]->moment.altitude = aircraft[2]->path->origin->altitude;
    aircraft[2]->moment.speed = aircraft[2]->path->origin->speed;
    aircraft[2]->arrivalTime_zulu = 1202;
    aircraft[2]->callsign = "UAL2";
//    aircraft[2]->graphic = new Graphic(Point(aircraft[2]->moment.currentPosition.longitude, aircraft[0]->moment.currentPosition.latitude, SpatialReference::wgs84()));
    m_mapView->graphicsOverlays()->append(aircraft[2]->graphicsOverlay);

    timer.startTime_zulu = 1200;
    timer.setWarpFactor(5);

    aircraft[0]->generateFuturePositions(timer, futurePositionStep);
    aircraft[1]->generateFuturePositions(timer, futurePositionStep);
    aircraft[2]->generateFuturePositions(timer, futurePositionStep);

    Aircraft *aircraft0 = aircraft[0];
    Aircraft *aircraft1 = aircraft[1];
    Aircraft *aircraft2 = aircraft[2];

    Conflict* aircraft01 = Conflict::detectConflictsBetweenAircraft(aircraft0, aircraft1, timer, futurePositionStep, &airspaceGraph);
    Conflict* aircraft12 = Conflict::detectConflictsBetweenAircraft(aircraft1, aircraft2, timer, futurePositionStep, &airspaceGraph);
    Conflict* aircraft20 = Conflict::detectConflictsBetweenAircraft(aircraft2, aircraft0, timer, futurePositionStep, &airspaceGraph);

    if (aircraft01 != nullptr) {
        cout << "Aircraft 1 has conflict with Aircraft 0" << endl;
    }
    if (aircraft12 != nullptr) {
        cout << "Aircraft 1 has conflict with Aircraft 2" << endl;
    }
    if (aircraft20 != nullptr) {
        cout << "Aircraft 2 has conflict with Aircraft 0" << endl;
    }

    std::pair<Aircraft*, Path> resolution = aircraft12->generatePathResolution(this);
    if (resolution.first == aircraft1) {
        aircraft1->setPath(&resolution.second);
    } else if (resolution.first == aircraft2) {
        aircraft2->setPath(&resolution.second);
    }

}

void GraphBasedTraconSimulator::addGraphicsOverlay() {
    // create the point geometry
    Point pointGeometry(40e5, 40e5, SpatialReference::wgs84());
    // create the point symbol
    SimpleMarkerSymbol* pointSymbol = new SimpleMarkerSymbol(SimpleMarkerSymbolStyle::Diamond, QColor("red"), 10, this);
    // Point graphic
    Graphic* pointGraphic = new Graphic(pointGeometry, this);

    // create a graphic overlay to display the point graphic
    GraphicsOverlay* pointGraphicOverlay = new GraphicsOverlay(this);
    // set the renderer of the overlay to be the marker symbol
    pointGraphicOverlay->setRenderer(new SimpleRenderer(pointSymbol, this));
    // add the graphic to the overlay
    pointGraphicOverlay->graphics()->append(pointGraphic);
    // add the overlay to the mapview
    m_mapView->graphicsOverlays()->append(pointGraphicOverlay);

    // create line geometry
    PolygonBuilder polylineBuilder(SpatialReference::webMercator());
    // build the polyline
    polylineBuilder.addPoint(-10e5, 40e5);
    polylineBuilder.addPoint(20e5, 50e5);
    // create a line symbol
    SimpleLineSymbol* sls = new SimpleLineSymbol(SimpleLineSymbolStyle::Solid, QColor("blue"), 5, this);
    // create a line graphic
    Graphic* lineGraphic = new Graphic(polylineBuilder.toGeometry(), this);

    // create a graphic overlay to display the line graphic
    GraphicsOverlay* lineGraphicOverlay = new GraphicsOverlay(this);
    // set the renderer of the graphic overlay to be the line symbol
    lineGraphicOverlay->setRenderer(new SimpleRenderer(sls, this));
    // add the graphic to the overlay
    lineGraphicOverlay->graphics()->append(lineGraphic);
    // add the overlay to the mapview
    m_mapView->graphicsOverlays()->append(lineGraphicOverlay);

    // create the polygon geometry
    PolygonBuilder polygonBuilder(SpatialReference::wgs84());
    polygonBuilder.addPoint(-117, 34);
    polygonBuilder.addPoint(-118, 34);
    polygonBuilder.addPoint(-118, 35);
    polygonBuilder.addPoint(-117, 35);
    SimpleFillSymbol* sfs = new SimpleFillSymbol(SimpleFillSymbolStyle::Solid, QColor(255, 255, 0, 180), this);
    Graphic* polygonGraphic = new Graphic(polygonBuilder.toGeometry(), this);

    GraphicsOverlay* polygonGraphicsOverlay = new GraphicsOverlay(this);
    polygonGraphicsOverlay->setRenderer(new SimpleRenderer(sfs, this));
    polygonGraphicsOverlay->graphics()->append(polygonGraphic);
    m_mapView->graphicsOverlays()->append(polygonGraphicsOverlay);
}
