#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

template <typename T_point, typename PointList>
PointList vectorToEigenArray(const T_point &vect) {
  const size_t nCols = vect.size();
  const size_t nRows = static_cast<size_t>(vect[0].rows());
  PointList res(nRows, nCols);
  for (size_t i = 0; i < vect.size(); ++i) {
    res.block(0, static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(nRows), 1) = vect[i];
  }
  return res;
}

using Vector2 = Eigen::Matrix<double, 2, 1>;
using Vector3 = Eigen::Matrix<double, 3, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using VectorN = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using Matrix3 = Eigen::Matrix<double, 3, 3>;
using Matrix4 = Eigen::Matrix<double, 4, 4>;

using Matrix34 = Eigen::Matrix<double, 3, 4>;
using Matrix43 = Eigen::Matrix<double, 4, 3>;
using Matrix3N = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using MatrixN = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using MatrixN_int = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;

#endif  // TYPES_H_INCLUDED
