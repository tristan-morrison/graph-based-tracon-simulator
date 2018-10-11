#include "FuturePosition.h"
#include "Position.h"

FuturePosition::FuturePosition(Position myPosition, int myNextFixIndex, int myPrevFixIndex)
{
    position = myPosition;
    nextFixIndex = myNextFixIndex;
    prevFixIndex = myPrevFixIndex;

}
