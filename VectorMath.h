#include <stdlib.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <math.h>

#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

namespace VectorMath {

  template <typename t>
  std::vector<t> static vecAdd(std::vector<t> vec1, std::vector<t> vec2) {
    std::vector<t> resultVec(vec1.size(), 0);
    for (int k = 0; k < vec1.size(); k++) {
      resultVec[k] = vec1[k] + vec2[k];
    }
    return resultVec;
  }

  template <typename t>
  std::vector<t> static vecSub(std::vector<t> vec1, std::vector<t> vec2) {
    std::vector<t> resultVec(vec1.size(), 0);
    for (int k = 0; k < vec1.size(); k++) {
      resultVec[k] = vec1[k] - vec2[k];
    }
    return resultVec;
  }

  template <typename t>
  double static vec2Radial (std::vector<t> vec) {
    return -atan2(vec[0], vec[1]);
  }

  template <typename t>
  double static vec2Len (std::vector<t> vec) {
    return sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]));
  }

  template <typename t>
  std::vector<t> static vecScale(std::vector<t> vec, t scaleFactor) {
    for (int k = 0; k < vec.size(); k++) {
      vec[k] *= scaleFactor;
    }
    return vec;
  }

  template <typename t>
  std::vector<t> static vectorizeHeading (double heading) {
    std::vector<t> resultVec(2, 0);
    resultVec[0] = sin(heading);
    resultVec[1] = cos(heading);
    return resultVec;
  }

}

#endif
