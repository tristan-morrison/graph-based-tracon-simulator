
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

import QtQuick 2.6
import QtQuick.Controls 1.4
import Esri.GraphBasedTraconSimulator 1.0

GraphBasedTraconSimulator {
    width: 800
    height: 600

    // Create MapQuickView here, and create its Map etc. in C++ code
    MapView {
        anchors.fill: parent
        objectName: "mapView"
        // set focus to enable keyboard navigation
        focus: true
    }
}
