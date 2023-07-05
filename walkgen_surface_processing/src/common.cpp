#include <common.hpp>

double min(const double &a, const double &b) { return a < b ? a : b; }

bool eq(const double &a, const double &b) { return std::abs(a - b) <= 1e-8; }

int wrap(const int &a, const int &b) {
  if (b == 1) {
    return 0;
  }
  return a < 0 ? a % b + b : a % b;
}

double srand(const double &min = 0, const double &max = 1) {
  return std::rand() / (double)RAND_MAX * (max - min) + min;
}
