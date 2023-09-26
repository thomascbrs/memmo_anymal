#include "FootTrajectoryPolynomial.hpp"

FootTrajectoryPolynomial::FootTrajectoryPolynomial(const FootTrajectoryPolynomial &other)
    : Ax(other.Ax),
      Ay(other.Ay),
      Az(other.Az),
      dt_(other.dt_),
      N_(other.N_),
      step_height_(other.step_height_),
      t0_(0.),
      M_current_(other.M_current_),
      M_next_(other.M_next_),
      M_tmp_(pinocchio::SE3::Identity()),
      Vel_tmp_(pinocchio::Motion::Zero()),
      t_swing_(other.t_swing_) {}

FootTrajectoryPolynomial::FootTrajectoryPolynomial(double dt, int N, double step_height,
                                                   const pinocchio::SE3 &M_current, const pinocchio::SE3 &M_next)
    : Ax(Vector6::Zero()),
      Ay(Vector6::Zero()),
      Az(Vector7::Zero()),
      dt_(dt),
      N_(N),
      step_height_(step_height),
      t0_(0.),
      M_current_(M_current),
      M_next_(M_next),
      M_tmp_(pinocchio::SE3::Identity()),
      Vel_tmp_(pinocchio::Motion::Zero()) {
  t_swing_ = dt_ * static_cast<double>(N_);

  double height_ = std::max(M_current_.translation()(2), M_next_.translation()(2));
  // Initale velocity and acceleration nulle
  updatePolyCoeff_XY(M_current_.translation(), Vector3::Zero(), Vector3::Zero(), M_next_.translation(), 0., t_swing_);
  // Update Z coefficients only at the beginning of the flying phase
  updatePolyCoeff_Z(M_current_.translation(), M_next_.translation(), t_swing_, step_height_ + height_);
}

FootTrajectoryPolynomial::~FootTrajectoryPolynomial() {}

pinocchio::SE3 FootTrajectoryPolynomial::position(int k) {
  if (k >= N_) {
    throw std::invalid_argument("Invalid argument: k is bigger than the allocated number of nodes.");
  }

  M_tmp_.translation() = evaluatePoly(0, k * dt_);
  return M_tmp_;
}

pinocchio::Motion FootTrajectoryPolynomial::velocity(int k) {
  if (k >= N_) {
    throw std::invalid_argument("Invalid argument: k is bigger than the allocated number of nodes.");
  }

  Vel_tmp_.linear() = evaluatePoly(1, k * dt_);
  return Vel_tmp_;
}

void FootTrajectoryPolynomial::update(const Vector3 &x0, const Vector3 &v0, const Vector3 &xf, double t0) {
  // v0 : not being used.
  t0_ = t0;
  if (t0 < 10e-4) {
    double height_ = std::max(x0(2), xf(2));
    // Update Z coefficients only at the beginning of the flying phase
    updatePolyCoeff_Z(x0, xf, t_swing_, step_height_ + height_);
    // Initale velocity and acceleration nulle
    updatePolyCoeff_XY(x0, Vector3::Zero(), Vector3::Zero(), xf, 0., t_swing_);
  } else {
    updatePolyCoeff_XY(x0, evaluatePoly(1, t0), evaluatePoly(2, t0), xf, t0, t_swing_);
  }
}

Vector3 FootTrajectoryPolynomial::evaluatePoly(int const &indice, double const &t) {
  Vector3 vector = Vector3::Zero();
  if (indice == 0) {
    double x = Ax(0) + Ax(1) * t + Ax(2) * std::pow(t, 2) + Ax(3) * std::pow(t, 3) + Ax(4) * std::pow(t, 4) +
               Ax(5) * std::pow(t, 5);
    double y = Ay(0) + Ay(1) * t + Ay(2) * std::pow(t, 2) + Ay(3) * std::pow(t, 3) + Ay(4) * std::pow(t, 4) +
               Ay(5) * std::pow(t, 5);
    double z = Az(0) + Az(1) * t + Az(2) * std::pow(t, 2) + Az(3) * std::pow(t, 3) + Az(4) * std::pow(t, 4) +
               Az(5) * std::pow(t, 5) + Az(6) * std::pow(t, 6);
    vector << x, y, z;
  }

  if (indice == 1) {
    double vx =
        Ax(1) + 2 * Ax(2) * t + 3 * Ax(3) * std::pow(t, 2) + 4 * Ax(4) * std::pow(t, 3) + 5 * Ax(5) * std::pow(t, 4);
    double vy =
        Ay(1) + 2 * Ay(2) * t + 3 * Ay(3) * std::pow(t, 2) + 4 * Ay(4) * std::pow(t, 3) + 5 * Ay(5) * std::pow(t, 4);
    double vz = Az(1) + 2 * Az(2) * t + 3 * Az(3) * std::pow(t, 2) + 4 * Az(4) * std::pow(t, 3) +
                5 * Az(5) * std::pow(t, 4) + 6 * Az(6) * std::pow(t, 5);
    vector << vx, vy, vz;
  }

  if (indice == 2) {
    double ax = 2 * Ax(2) + 6 * Ax(3) * t + 12 * Ax(4) * std::pow(t, 2) + 20 * Ax(5) * std::pow(t, 3);
    double ay = 2 * Ay(2) + 6 * Ay(3) * t + 12 * Ay(4) * std::pow(t, 2) + 20 * Ay(5) * std::pow(t, 3);
    double az = 2 * Az(2) + 6 * Az(3) * t + 12 * Az(4) * std::pow(t, 2) + 20 * Az(5) * std::pow(t, 3) +
                30 * Az(6) * std::pow(t, 4);
    vector << ax, ay, az;
  }

  return vector;
}

void FootTrajectoryPolynomial::updatePolyCoeff_XY(Vector3 const &x_init, Vector3 const &v_init, Vector3 const &a_init,
                                                  Vector3 const &x_target, double const &t0, double const &t1) {
  double x0 = x_init(0);
  double y0 = x_init(1);
  double dx0 = v_init(0);
  double dy0 = v_init(1);
  double ddx0 = a_init(0);
  double ddy0 = a_init(1);
  double x1 = x_target(0);
  double y1 = x_target(1);

  double d = t1;
  double t = t0;

  // Compute polynoms coefficients for x and y
  Ax(5) =
      (ddx0 * std::pow(t, 2) - 2 * ddx0 * t * d - 6 * dx0 * t + ddx0 * std::pow(d, 2) + 6 * dx0 * d + 12 * x0 -
       12 * x1) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));

  Ax(4) =
      (30 * t * x1 - 30 * t * x0 - 30 * d * x0 + 30 * d * x1 - 2 * std::pow(t, 3) * ddx0 - 3 * std::pow(d, 3) * ddx0 +
       14 * std::pow(t, 2) * dx0 - 16 * std::pow(d, 2) * dx0 + 2 * t * d * dx0 + 4 * t * std::pow(d, 2) * ddx0 +
       std::pow(t, 2) * d * ddx0) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ax(3) =
      (std::pow(t, 4) * ddx0 + 3 * std::pow(d, 4) * ddx0 - 8 * std::pow(t, 3) * dx0 + 12 * std::pow(d, 3) * dx0 +
       20 * std::pow(t, 2) * x0 - 20 * std::pow(t, 2) * x1 + 20 * std::pow(d, 2) * x0 - 20 * std::pow(d, 2) * x1 +
       80 * t * d * x0 - 80 * t * d * x1 + 4 * std::pow(t, 3) * d * ddx0 + 28 * t * std::pow(d, 2) * dx0 -
       32 * std::pow(t, 2) * d * dx0 - 8 * std::pow(t, 2) * std::pow(d, 2) * ddx0) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ax(2) = -(std::pow(d, 5) * ddx0 + 4 * t * std::pow(d, 4) * ddx0 + 3 * std::pow(t, 4) * d * ddx0 +
            36 * t * std::pow(d, 3) * dx0 - 24 * std::pow(t, 3) * d * dx0 + 60 * t * std::pow(d, 2) * x0 +
            60 * std::pow(t, 2) * d * x0 - 60 * t * std::pow(d, 2) * x1 - 60 * std::pow(t, 2) * d * x1 -
            8 * std::pow(t, 2) * std::pow(d, 3) * ddx0 - 12 * std::pow(t, 2) * std::pow(d, 2) * dx0) /
          (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) *
           (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ax(1) =
      -(2 * std::pow(d, 5) * dx0 - 2 * t * std::pow(d, 5) * ddx0 - 10 * t * std::pow(d, 4) * dx0 +
        std::pow(t, 2) * std::pow(d, 4) * ddx0 + 4 * std::pow(t, 3) * std::pow(d, 3) * ddx0 -
        3 * std::pow(t, 4) * std::pow(d, 2) * ddx0 - 16 * std::pow(t, 2) * std::pow(d, 3) * dx0 +
        24 * std::pow(t, 3) * std::pow(d, 2) * dx0 - 60 * std::pow(t, 2) * std::pow(d, 2) * x0 +
        60 * std::pow(t, 2) * std::pow(d, 2) * x1) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ax(0) = (2 * x1 * std::pow(t, 5) - ddx0 * std::pow(t, 4) * std::pow(d, 3) - 10 * x1 * std::pow(t, 4) * d +
           2 * ddx0 * std::pow(t, 3) * std::pow(d, 4) + 8 * dx0 * std::pow(t, 3) * std::pow(d, 3) +
           20 * x1 * std::pow(t, 3) * std::pow(d, 2) - ddx0 * std::pow(t, 2) * std::pow(d, 5) -
           10 * dx0 * std::pow(t, 2) * std::pow(d, 4) - 20 * x0 * std::pow(t, 2) * std::pow(d, 3) +
           2 * dx0 * t * std::pow(d, 5) + 10 * x0 * t * std::pow(d, 4) - 2 * x0 * std::pow(d, 5)) /
          (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) *
           (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));

  Ay(5) =
      (ddy0 * std::pow(t, 2) - 2 * ddy0 * t * d - 6 * dy0 * t + ddy0 * std::pow(d, 2) + 6 * dy0 * d + 12 * y0 -
       12 * y1) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ay(4) =
      (30 * t * y1 - 30 * t * y0 - 30 * d * y0 + 30 * d * y1 - 2 * std::pow(t, 3) * ddy0 - 3 * std::pow(d, 3) * ddy0 +
       14 * std::pow(t, 2) * dy0 - 16 * std::pow(d, 2) * dy0 + 2 * t * d * dy0 + 4 * t * std::pow(d, 2) * ddy0 +
       std::pow(t, 2) * d * ddy0) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ay(3) =
      (std::pow(t, 4) * ddy0 + 3 * std::pow(d, 4) * ddy0 - 8 * std::pow(t, 3) * dy0 + 12 * std::pow(d, 3) * dy0 +
       20 * std::pow(t, 2) * y0 - 20 * std::pow(t, 2) * y1 + 20 * std::pow(d, 2) * y0 - 20 * std::pow(d, 2) * y1 +
       80 * t * d * y0 - 80 * t * d * y1 + 4 * std::pow(t, 3) * d * ddy0 + 28 * t * std::pow(d, 2) * dy0 -
       32 * std::pow(t, 2) * d * dy0 - 8 * std::pow(t, 2) * std::pow(d, 2) * ddy0) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ay(2) = -(std::pow(d, 5) * ddy0 + 4 * t * std::pow(d, 4) * ddy0 + 3 * std::pow(t, 4) * d * ddy0 +
            36 * t * std::pow(d, 3) * dy0 - 24 * std::pow(t, 3) * d * dy0 + 60 * t * std::pow(d, 2) * y0 +
            60 * std::pow(t, 2) * d * y0 - 60 * t * std::pow(d, 2) * y1 - 60 * std::pow(t, 2) * d * y1 -
            8 * std::pow(t, 2) * std::pow(d, 3) * ddy0 - 12 * std::pow(t, 2) * std::pow(d, 2) * dy0) /
          (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) *
           (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ay(1) =
      -(2 * std::pow(d, 5) * dy0 - 2 * t * std::pow(d, 5) * ddy0 - 10 * t * std::pow(d, 4) * dy0 +
        std::pow(t, 2) * std::pow(d, 4) * ddy0 + 4 * std::pow(t, 3) * std::pow(d, 3) * ddy0 -
        3 * std::pow(t, 4) * std::pow(d, 2) * ddy0 - 16 * std::pow(t, 2) * std::pow(d, 3) * dy0 +
        24 * std::pow(t, 3) * std::pow(d, 2) * dy0 - 60 * std::pow(t, 2) * std::pow(d, 2) * y0 +
        60 * std::pow(t, 2) * std::pow(d, 2) * y1) /
      (2 * std::pow((t - d), 2) * (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
  Ay(0) = (2 * y1 * std::pow(t, 5) - ddy0 * std::pow(t, 4) * std::pow(d, 3) - 10 * y1 * std::pow(t, 4) * d +
           2 * ddy0 * std::pow(t, 3) * std::pow(d, 4) + 8 * dy0 * std::pow(t, 3) * std::pow(d, 3) +
           20 * y1 * std::pow(t, 3) * std::pow(d, 2) - ddy0 * std::pow(t, 2) * std::pow(d, 5) -
           10 * dy0 * std::pow(t, 2) * std::pow(d, 4) - 20 * y0 * std::pow(t, 2) * std::pow(d, 3) +
           2 * dy0 * t * std::pow(d, 5) + 10 * y0 * t * std::pow(d, 4) - 2 * y0 * std::pow(d, 5)) /
          (2 * (std::pow(t, 2) - 2 * t * d + std::pow(d, 2)) *
           (std::pow(t, 3) - 3 * std::pow(t, 2) * d + 3 * t * std::pow(d, 2) - std::pow(d, 3)));
}

void FootTrajectoryPolynomial::updatePolyCoeff_Z(Vector3 const &x_init, Vector3 const &x_target, double const &t1,
                                                 double const &h) {
  double z0 = x_init(2);
  double z1 = x_target(2);

  //  Version 3D (z1 != 0)
  Az(6) = (32. * z0 + 32. * z1 - 64. * h) / std::pow(t1, 6);
  Az(5) = -(102. * z0 + 90. * z1 - 192. * h) / std::pow(t1, 5);
  Az(4) = (111. * z0 + 81. * z1 - 192. * h) / std::pow(t1, 4);
  Az(3) = -(42. * z0 + 22. * z1 - 64. * h) / std::pow(t1, 3);
  Az(2) = 0;
  Az(1) = 0;
  Az(0) = z0;
}
