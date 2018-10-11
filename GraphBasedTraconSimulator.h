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

#ifndef GRAPHBASEDTRACONSIMULATOR_H
#define GRAPHBASEDTRACONSIMULATOR_H

namespace Esri
{
namespace ArcGISRuntime
{
class Map;
class MapQuickView;
}
}

#include <QQuickItem>
#include <QTimer>
#include <QElapsedTimer>

#include "AirspaceGraph.h"
#include "Aircraft.h"
#include "Timer.h"

class GraphBasedTraconSimulator : public QQuickItem
{
    Q_OBJECT

public:
    GraphBasedTraconSimulator(QQuickItem* parent = nullptr);
    ~GraphBasedTraconSimulator();
    AirspaceGraph airspaceGraph;
    std::vector<Aircraft*> aircraft;
    Timer timer;
    QElapsedTimer elapsedTimer;
    QTimer loopTimer;
    int startTime_ms;
    int startTime_zulu;
    int futurePositionStep;
//    Esri::ArcGISRuntime::GraphicsOverlay *aircraftGraphicsOverlay;
    Esri::ArcGISRuntime::GraphicsOverlay *airspaceGraphGraphicsOverlay;
    Esri::ArcGISRuntime::GraphicsOverlay *pathsGraphicsOverlay;

    void componentComplete() override;

private:
    Esri::ArcGISRuntime::Map*             m_map = nullptr;
    Esri::ArcGISRuntime::MapQuickView*    m_mapView = nullptr;
    void addGraphicsOverlay();
    void loadAirspaceGraph();

private slots:
    void iterateMainLoop();

};

#endif // GRAPHBASEDTRACONSIMULATOR_H
