#ifndef _COMMON_H
#define _COMMON_H

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <cmath>
#include <ctime>
#include <cstdlib>

#define PI 3.1415926535897932384626433832795

bool eq(const double &a, double const &b);
double min(const double &a, const double &b);
int wrap(const int &a, const int &b);
double srand(const double &min, const double &max);

template <class T>
T &at(std::vector<T> v, int i) {
  return v.at(static_cast<size_t>(wrap(i, static_cast<int>(v.size()))));
};

#endif
