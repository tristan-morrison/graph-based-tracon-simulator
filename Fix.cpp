#include <stdio.h>
#include <string>
#include <climits>

#include "Fix.h"
// #ifndef POSITION_H
// #include "Position.h"
// #endif

Fix::Fix() {
  name = "";
  position = new Position(0, 0);
  type = "RNAV";
  indexInGraph = INT_MAX;
}

Fix::Fix (string myName, double latitude, double longitude, string myType) {
  name = myName;
  position = new Position(latitude, longitude);
  type = myType;
  indexInGraph = INT_MAX;
}
