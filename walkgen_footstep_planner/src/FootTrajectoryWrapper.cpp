#include <FootTrajectoryWrapper.hpp>

FootTrajectoryWrapper::~FootTrajectoryWrapper() {}

FootTrajectoryWrapper::FootTrajectoryWrapper(double dt, int N, double step_height, const pinocchio::SE3 &M_current,
                                             const pinocchio::SE3 &M_next)
    : USE_POLY(false),  // By default, use bezier curves.
      dt_(dt),
      N_(N),
      step_height_(step_height),
      M_current_(M_current),
      M_next_(M_next),
      M_tmp_(pinocchio::SE3::Identity()),
      Vel_tmp_(pinocchio::Motion::Zero()) {
  initialize_curve();
}

FootTrajectoryWrapper::FootTrajectoryWrapper(double dt, int N, double step_height, const pinocchio::SE3 &M_current,
                                             const pinocchio::SE3 &M_next, const Params &params)
    : USE_POLY(false),  // By default, use bezier curves.
      dt_(dt),
      N_(N),
      step_height_(step_height),
      M_current_(M_current),
      M_next_(M_next),
      M_tmp_(pinocchio::SE3::Identity()),
      Vel_tmp_(pinocchio::Motion::Zero()),
      params_(params) {
  initialize_curve();
}

FootTrajectoryWrapper::FootTrajectoryWrapper(double dt, int N, double step_height, const pinocchio::SE3 &M_current,
                                             const pinocchio::SE3 &M_next, bool use_poly)
    : USE_POLY(use_poly),  // By default, use bezier curves.
      dt_(dt),
      N_(N),
      step_height_(step_height),
      M_current_(M_current),
      M_next_(M_next),
      M_tmp_(pinocchio::SE3::Identity()),
      Vel_tmp_(pinocchio::Motion::Zero()) {
  initialize_curve();
}

FootTrajectoryWrapper::FootTrajectoryWrapper(double dt, int N, double step_height, const pinocchio::SE3 &M_current,
                                             const pinocchio::SE3 &M_next, const Params &params, bool use_poly)
    : USE_POLY(use_poly),  // By default, use bezier curves.
      dt_(dt),
      N_(N),
      step_height_(step_height),
      M_current_(M_current),
      M_next_(M_next),
      M_tmp_(pinocchio::SE3::Identity()),
      Vel_tmp_(pinocchio::Motion::Zero()),
      params_(params) {
  initialize_curve();
}

void FootTrajectoryWrapper::initialize_curve() {
  if (USE_POLY) {
    poly_ = std::make_shared<FootTrajectoryPolynomial>(dt_, N_, step_height_, M_current_, M_next_);
  } else {
    double t_swing = params_.dt * static_cast<double>(N_);
    curve_ = std::make_shared<FootTrajectoryBezier>();
    curve_->initialize(params_.N_sample, params_.N_sample_ineq, params_.degree, t_swing, step_height_);
    curve_->set_parameters_up(params_.margin_up, params_.t_margin_up, params_.z_margin_up);
    curve_->set_parameters_down(params_.margin_down, params_.t_margin_down, params_.z_margin_down);
    curve_->create_simple_curve(M_current_.translation(), Vector3::Zero(), M_next_.translation(), 0.);
  }
}

pinocchio::SE3 FootTrajectoryWrapper::position(int k) {
  if (k >= N_) {
    throw std::invalid_argument("Invalid argument: k is bigger than the allocated number of nodes.");
  }
  if (USE_POLY) {
    M_tmp_.translation() = poly_->evaluatePoly(0, k * dt_);
  } else {
    M_tmp_.translation() = curve_->evaluateBezier(0, k * dt_);
  }
  return M_tmp_;
}

pinocchio::Motion FootTrajectoryWrapper::velocity(int k) {
  if (k >= N_) {
    throw std::invalid_argument("Invalid argument: k is bigger than the allocated number of nodes.");
  }
  if (USE_POLY) {
    Vel_tmp_.linear() = poly_->evaluatePoly(0, k * dt_);
  } else {
    Vel_tmp_.linear() = curve_->evaluateBezier(1, k * dt_);
  }
  return Vel_tmp_;
}

void FootTrajectoryWrapper::update(const Vector3 &x0, const Vector3 &v0, const Vector3 &xf, double t0,
                                   const Surface &init_surface, const Surface &end_surface) {
  if (USE_POLY) {
    poly_->update(x0, v0, xf, t0);
  } else {
    curve_->update(x0, v0, xf, t0, init_surface, end_surface);
  }
};

MatrixN FootTrajectoryWrapper::get_coefficients() {
  if (USE_POLY) {
    throw std::runtime_error("Cannot return the trajectory coefficient when using the POLYNOMIAL mode.");
  }
  return curve_->getCoefficients();
}

std::shared_ptr<FootTrajectoryBezier> FootTrajectoryWrapper::get_curve() {
  if (USE_POLY) {
    throw std::runtime_error("Cannot return the bezier curve object when using the POLYNOMIAL mode.");
  }
  return curve_;
}

// Copy constructor implementation
FootTrajectoryWrapper::FootTrajectoryWrapper(const FootTrajectoryWrapper &other) {
  // Copy all data members
  USE_POLY = other.USE_POLY;
  dt_ = other.dt_;
  N_ = other.N_;
  step_height_ = other.step_height_;
  M_current_ = other.M_current_;
  M_next_ = other.M_next_;
  M_tmp_ = other.M_tmp_;
  Vel_tmp_ = other.Vel_tmp_;
  // std::shared_ptr<FootTrajectoryBezier> curve_cp =
  // std::make_shared<FootTrajectoryBezier>(*other.curve_); curve_ =
  // std::move(curve_cp);
  params_ = other.params_;
  if (USE_POLY) {
    poly_ = std::make_shared<FootTrajectoryPolynomial>(*other.poly_);
  } else {
    curve_ = std::make_shared<FootTrajectoryBezier>(*other.curve_);
  }
};

double FootTrajectoryWrapper::getT0() {
  if (USE_POLY) {
    return poly_->getT0();
  } else {
    return curve_->getT0();
  }
}

bool FootTrajectoryWrapper::getUsePoly() { return USE_POLY; }
