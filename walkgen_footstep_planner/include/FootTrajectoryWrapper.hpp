#include "FootTrajectoryBezier.hpp"
#include "Params.hpp"
#include "Surface.hpp"
#include <Eigen/Core>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>

#ifndef FOOT_TRAJECTORY_WRAPPER_HPP
#define FOOT_TRAJECTORY_WRAPPER_HPP

class FootTrajectoryWrapper {
public:
  friend class FootTrajectoryWrapper;
  FootTrajectoryWrapper(const FootTrajectoryWrapper &other);

  // Constructor without parameter.
  FootTrajectoryWrapper(double dt, int N, double step_height,
                        const pinocchio::SE3 &M_current,
                        const pinocchio::SE3 &M_next);

  // Constructor with parameter object.
  FootTrajectoryWrapper(double dt, int N, double step_height,
                        const pinocchio::SE3 &M_current,
                        const pinocchio::SE3 &M_next, const Params &params);

  // Destructor
  ~FootTrajectoryWrapper();

  pinocchio::SE3 position(int k);
  pinocchio::Motion velocity(int k);
  void update(const Vector3 &x0, const Vector3 &v0, const Vector3 &xf,
              double t0, const Surface &init_surface,
              const Surface &end_surface);
  MatrixN get_coefficients();
  std::shared_ptr<FootTrajectoryBezier> get_curve();
  double getT0();
  int flag = 0;

private:
  double dt_;
  int N_;
  double step_height_;
  pinocchio::SE3 M_current_;
  pinocchio::SE3 M_next_;
  pinocchio::SE3 M_tmp_;
  pinocchio::Motion Vel_tmp_;

  std::shared_ptr<FootTrajectoryBezier> curve_ =
      std::make_shared<FootTrajectoryBezier>();
  Params params_ = Params();
};

#endif // FOOT_TRAJECTORY_WRAPPER_HPP
