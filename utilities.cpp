#include <stdlib.h>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <string>

#include "utilities.h"

using namespace std;

template <typename T> void printVector (vector<T> vec, string label) {
  cout << label << ":" << endl;

  for (int k = 0; k < vec.size(); k++) {
    cout << vec[k];

    if (k < vec.size()-1) {
      cout << ", ";
    }
  }
  cout << endl;
}

template <typename T> void printtwodVector (vector<vector<T> > vec, string label) {
  cout << label << ":" << endl;

  for (int k = 0; k < vec.size(); k++) {
    printVector(vec[k], "Row " + k);
  }

}
