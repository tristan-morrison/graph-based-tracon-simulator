#include "Graphic.h"

#include <stdlib.h>
#include <vector>

#include "Fix.h"

using namespace std;

/* -------------------------------------------

A sequence of fixes

----------------------------------------------*/

#ifndef PATH_H
#define PATH_H

class Path {
  public:
    // data members
    vector<Fix*> fixSequence;
    Fix *origin, *destination;

    // function members
    Path(Fix* myOrigin, Fix* myDestination);
//    Path(Path &pathToCopy);
    double distanceToDestination_nm(Position* currentPosition, int nextFix_index);
    Esri::ArcGISRuntime::Graphic * generateGraphic(QObject* parent);

};

#endif
