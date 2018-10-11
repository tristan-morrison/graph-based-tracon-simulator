#include <stdio.h>
#include <string>

#ifndef FIX_H
#define FIX_H

#include "Position.h"

class Fix {
  public:
    Fix();
    Fix (string myName, double latitude, double longitude, string myType);
    Position * position;
    string name;
    string type;
    int indexInGraph;
    int altitude;
    int speed;

};

#endif
