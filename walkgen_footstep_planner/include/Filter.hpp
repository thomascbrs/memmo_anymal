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

  FilterMean(double period, double dt)
      : _Nx(static_cast<size_t>(int(period / dt))) {
    _x_queue.reserve(_Nx);
  }

  Eigen::VectorXd filter(const Eigen::VectorXd &q) {
    // Check size of input
    if (q.size() != 6) {
      throw std::invalid_argument("q should be size 6");
    }

    if (_x_queue.size() == _Nx) {
      _x_queue.erase(_x_queue.begin());
    }

    // Handle modulo for orientation
    if (!_x_queue.empty() &&
        std::abs(q(5) - _x_queue.front()(5)) > 1.5 * M_PI) {
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

#endif // FILTERMEAN_H_INCLUDED
