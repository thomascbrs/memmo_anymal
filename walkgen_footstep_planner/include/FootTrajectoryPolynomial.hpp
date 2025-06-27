#include <Eigen/Core>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "Types.hpp"


#ifndef FOOT_TRAJECTORY_POLYNOMIAL_HPP
#define FOOT_TRAJECTORY_POLYNOMIAL_HPP

class FootTrajectoryPolynomial {
 public:
  FootTrajectoryPolynomial(const FootTrajectoryPolynomial &other);

  // Constructor without parameter.
  FootTrajectoryPolynomial(double dt, int N, double step_height, const pinocchio::SE3 &M_current,
                        const pinocchio::SE3 &M_next);

  // Destructor
  ~FootTrajectoryPolynomial();

  Vector3 evaluatePoly(int const &indice, double const &t);

  void updatePolyCoeff_XY(Vector3 const &x_init, Vector3 const &v_init, Vector3 const &a_init, Vector3 const &x_target,
                          double const &t0, double const &t1);
  void updatePolyCoeff_Z(Vector3 const &x_init, Vector3 const &x_target, double const &t1, double const &h);

  // Operator ==, necessary to register a list
  bool operator==(const FootTrajectoryPolynomial &other) const {
    return Ax == other.Ax && Ay == other.Ay && Az == other.Az && N_ == other.N_ && dt_ == other.dt_;
  }

  pinocchio::SE3 position(int k);
  pinocchio::Motion velocity(int k);
  void update(const Vector3 &x0, const Vector3 &v0, const Vector3 &xf, double t0);
  double getT0() { return t0_; };

 private:
  Vector6 Ax;  ///< Coefficients for the X component
  Vector6 Ay;  ///< Coefficients for the Y component
  Vector7 Az;  ///< Coefficients for the Z component

  double dt_;
  int N_;
  double step_height_;
  double t0_;
  pinocchio::SE3 M_current_;
  pinocchio::SE3 M_next_;
  pinocchio::SE3 M_tmp_;
  pinocchio::Motion Vel_tmp_;
  double t_swing_;
};

#endif  // FOOT_TRAJECTORY_POLYNOMIAL_HPP
