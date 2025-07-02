#ifndef FILTERMEAN_H_INCLUDED
#define FILTERMEAN_H_INCLUDED

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <vector>

class FilterMean {
 public:
  // Default constructor
  FilterMean(){};

  FilterMean(double period, double dt) : _Nx(static_cast<size_t>(int(period / dt))) { _x_queue.reserve(_Nx); }

  Eigen::VectorXd filter(const Eigen::VectorXd &q) {
    // Check size of input
    if (q.size() != 6) {
      throw std::invalid_argument("q should be size 6");
    }

    if (_x_queue.size() == _Nx) {
      _x_queue.erase(_x_queue.begin());
    }

    // Handle modulo for orientation
    if (!_x_queue.empty() && std::abs(q(5) - _x_queue.front()(5)) > 1.5 * M_PI) {
      handle_modulo(q(5) - _x_queue.front()(5) > 0);
    }

    _x_queue.push_back(q);
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(6);
    for (const auto &x : _x_queue) {
      sum += x;
    }

    return sum / _x_queue.size();
  }

 private:
  void handle_modulo(bool dir) {
    const double delta_yaw = dir ? 2.0 * M_PI : -2.0 * M_PI;
    for (auto &x : _x_queue) {
      x(5) += delta_yaw;
    }
  }

  size_t _Nx;
  std::vector<Eigen::VectorXd> _x_queue;
};

class FilterIntegrator {
 public:
  // Default constructor
  FilterIntegrator(){};

  /// @param dt   Integration timestep [s]
  FilterIntegrator(double dt, const Eigen::VectorXd &q) 
    : _dt(dt),
      _q(q)
  {}

  /// Integrate velocity vector v = [vx, vy, vz, wx, wy, wz]
  /// into pose q = [x, y, z, roll, pitch, yaw].
  Eigen::VectorXd filter(const Eigen::VectorXd &v) {
    if (v.size() != 6)
      throw std::invalid_argument("FilterIntegrator: v must be size 6");

    double vx = v(0), vy = v(1), vz = v(2);
    double wz = v(5);

    // Integrate XY with exact unicycle formula if rotating,
    // otherwise simple Euler:
    if (std::abs(wz) > 1e-2) {
      double theta = wz * _dt;
      double sin_t = std::sin(theta);
      double cos_t = std::cos(theta);
      double dx = (vx * sin_t + vy * (cos_t - 1.0)) / wz;
      double dy = (vy * sin_t - vx * (cos_t - 1.0)) / wz;
      _q(0) += dx;
      _q(1) += dy;
    } else {
      _q(0) += vx * _dt;
      _q(1) += vy * _dt;
    }

    // Linear Z (not used in our specific case)
    // _q(2) += vz * _dt;

    // Yaw integration (deal with modulo not necessary in this case)
    _q(5) += wz * _dt;

    // Keep roll & pitch at zero (or extend if needed)
    _q(3) = 0.0;
    _q(4) = 0.0;

    return _q;
  }

 private:
  double _dt;
  Eigen::VectorXd _q;  // [x, y, z, roll, pitch, yaw]
};

#endif  // FILTERMEAN_H_INCLUDED
