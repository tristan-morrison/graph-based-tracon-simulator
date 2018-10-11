#ifndef FUTUREPOSITION_H
#define FUTUREPOSITION_H

#include "Position.h"

class FuturePosition
{
public:
    Position position;
    int nextFixIndex;
    int prevFixIndex;

    FuturePosition(Position myPosition, int myNextFixIndex, int myPrevFixIndex);
};

#endif // FUTUREPOSITION_H
