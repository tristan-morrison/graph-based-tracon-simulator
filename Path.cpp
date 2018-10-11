#include "PolylineBuilder.h"
#include "SpatialReference.h"
#include "Point.h"
#include "LineSegment.h"
#include "Part.h"
#include "PartCollection.h"
#include "Graphic.h"

#include <stdlib.h>
#include <vector>

#include "Path.h"

Path::Path(Fix* myOrigin, Fix* myDestination) {
  // fixSequence = new vector<Fix>();
  origin = myOrigin;
  destination = myDestination;
}

// TODO: Currently this function assumes that the aircraft has passed its origin. While this is generally a safe assumption, it should not be made. Fix.
double Path::distanceToDestination_nm(Position* currentPosition, int nextFix_index) {
    double distance = currentPosition->distanceTo(*(fixSequence[nextFix_index]->position));
    for (int k = nextFix_index; k < fixSequence.size() - 1; k++) {
        distance += fixSequence[nextFix_index]->position->distanceTo(*(fixSequence[nextFix_index + 1]->position));
    }
    distance += destination->position->distanceTo(*(fixSequence[fixSequence.size() - 1]->position));
    return distance;
}

//Path::Path (Path &pathToCopy) {
//    origin = pathToCopy.origin;
//    destination = pathToCopy.destination;
//    fixSequence = pathToCopy.fixSequence;
//}

using namespace Esri::ArcGISRuntime;

Graphic* Path::generateGraphic(QObject* parent) {
    PolylineBuilder* polylineBuilder = new PolylineBuilder(SpatialReference(4326), nullptr);
    PartCollection* partCollection = new PartCollection(SpatialReference(4326), nullptr);

    LineSegment firstSegment(origin->position->longitude, origin->position->latitude, fixSequence[0]->position->longitude, fixSequence[0]->position->latitude, SpatialReference(4326));
    Part * firstPart = new Part(SpatialReference(4326));
    firstPart->addSegment(firstSegment);
    partCollection->addPart(firstPart);

    for (int k = 0; k < fixSequence.size() - 1; k++) {
        LineSegment segmentToAdd(fixSequence[k]->position->longitude, fixSequence[k]->position->latitude, fixSequence[k+1]->position->longitude, fixSequence[k+1]->position->latitude, SpatialReference::wgs84());
        Part *partToAdd = new Part(SpatialReference(4326));
        partToAdd->addSegment(segmentToAdd);
        partCollection->addPart(partToAdd);
    }

    int lastIndex = fixSequence.size() - 1;
    LineSegment lastSegment(fixSequence[lastIndex]->position->longitude, fixSequence[lastIndex]->position->latitude, destination->position->longitude, destination->position->latitude, SpatialReference(4326));
    Part * lastPart = new Part(SpatialReference(4326));
    lastPart->addSegment(lastSegment);
    partCollection->addPart(lastPart);

    polylineBuilder->setParts(partCollection);
    Graphic * graphicToReturn = new Graphic(polylineBuilder->toGeometry());

    return graphicToReturn;
}
