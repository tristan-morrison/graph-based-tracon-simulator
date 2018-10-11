#-------------------------------------------------
#  Copyright 2016 ESRI
#
#  All rights reserved under the copyright laws of the United States
#  and applicable international laws, treaties, and conventions.
#
#  You may freely redistribute and use this sample code, with or
#  without modification, provided you include the original copyright
#  notice and use restrictions.
#
#  See the Sample code usage restrictions document for further information.
#-------------------------------------------------

TEMPLATE = app

QT += core gui opengl network positioning sensors qml quick
CONFIG += c++11

TARGET = test

equals(QT_MAJOR_VERSION, 5) {
    lessThan(QT_MINOR_VERSION, 9) {
        error("$$TARGET requires Qt 5.9.2")
    }
        equals(QT_MINOR_VERSION, 9) : lessThan(QT_PATCH_VERSION, 2) {
                error("$$TARGET requires Qt 5.9.2")
        }
}

ARCGIS_RUNTIME_VERSION = 100.3
include($$PWD/arcgisruntime.pri)

# Input
HEADERS += Aircraft.h \
           AircraftMoment.h \
           AircraftTarget.h \
           AirspaceGraph.h \
           AppInfo.h \
           constants.h \
           CSVRow.hpp \
           Fix.h \
           GraphBasedTraconSimulator.h \
           Path.h \
           Position.h \
           Timer.h \
           trigMath.h \
           unitConverters.h \
           utilities.h \
           VectorMath.h \
           CSVRow_QFile.h \
    Conflict.h \
    FuturePosition.h
SOURCES += Aircraft.cpp \
           AircraftMoment.cpp \
           AircraftTarget.cpp \
           AirspaceGraph.cpp \
           Fix.cpp \
           GraphBasedTraconSimulator.cpp \
           main.cpp \
           Path.cpp \
           Position.cpp \
           Timer.cpp \
           utilities.cpp \
    Conflict.cpp \
    FuturePosition.cpp
RESOURCES += qml/qml.qrc Resources/Resources.qrc \
    zla.qrc

#-------------------------------------------------------------------------------

win32 {
    include (Win/Win.pri)
}

macx {
    include (Mac/Mac.pri)
}

ios {
    include (iOS/iOS.pri)
}

android {
    include (Android/Android.pri)
}
