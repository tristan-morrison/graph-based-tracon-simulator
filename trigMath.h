#include <stdlib.h>
#include <math.h>

#ifndef TRIG_MATH_H
#define TRIG_MATH_H

namespace TrigMath {

  static const float TAU = M_PI * 2;

  float static degreesToRadians(float degrees) {
    return (degrees * M_PI) / 180;
  }

  float static radiansToDegrees(float radians) {
    return radians * (180 / M_PI);
  }

  // normalize angles to within 0 - 2π
  float static normalize_radians(float radians) {
    if (radians >= 0) {
      return fmod(radians, TAU);
    }

    return TAU + fmod(radians, TAU);
  }

  float static angleOffset (float a, float b) {
    a = radiansToDegrees(a);
    b = radiansToDegrees(b);
    bool invert = false;

    if (b > a) {
      invert = true;
      const float temp = a;

      a = b;
      b = temp;
    }

    float offset = fmod((a - b), 360);
    if (offset > 180) {
      offset -= 360;
    }

    if (invert) {
      offset *= -1;
    }

    offset = degreesToRadians(offset);

    return offset;

  }

}

#endif
