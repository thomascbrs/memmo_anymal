#include "FootTrajectoryBezier.hpp"

FootTrajectoryBezier::FootTrajectoryBezier()
    : intersectionPoint_(Vector2::Zero()),
      Ax(Vector6::Zero()),
      Ay(Vector6::Zero()),
      Az(Vector7::Zero()),
      t_swing_(0.),
      ineq_(Vector3::Zero()),
      ineq_vector_(0.),
      x_margin_(0.),
      t_stop_(0.),
      useBezier(true) {
  pDef.degree = 7;
  pDef.flag = optimization::INIT_POS | optimization::END_POS | optimization::INIT_VEL | optimization::END_VEL |
              optimization::INIT_ACC | optimization::END_ACC;
  Vector6 vector = Vector6::Zero();
  problem_data_t pbData = optimization::setup_control_points<pointX_t, double, safe>(pDef);
  bezier_linear_variable_t* bez = pbData.bezier;

  fitBezier = evaluateLinear<bezier_t, bezier_linear_variable_t>(*bez, vector);
}

void FootTrajectoryBezier::initialize(double x_margin_max_in, double t_margin_in, double z_margin_in,
                                               int N_samples_in, int N_samples_ineq_in, int degree_in, double t_swing, double maxHeight) {
  N_samples = N_samples_in;
  N_samples_ineq = N_samples_ineq_in;
  degree = degree_in;
  res_size = dim * (degree + 1 - 6);

  P_ = MatrixN::Zero(res_size, res_size);
  q_ = VectorN::Zero(res_size);
  C_ = MatrixN::Zero(res_size, 0);
  d_ = VectorN::Zero(0);
  x = VectorN::Zero(res_size);
  G_ = MatrixN::Zero(N_samples_ineq, res_size);
  h_ = VectorN::Zero(N_samples_ineq);
  x_margin_max_ = x_margin_max_in;
  t_margin_ = t_margin_in;
  z_margin_ = z_margin_in;
  t_swing_ = t_swing;
  maxHeight_ = maxHeight;
}

void FootTrajectoryBezier::create_simple_curve(Vector3 const& pos_init, Vector3 const& vel_init, Vector3 const& pos_end, double t0){
  double delta_t = t_swing_ - t0;
  t0_ = t0;
  if (t0 < 10e-4) {
    double height_ = std::max(pos_init(2), pos_end(2));
    // Update Z coefficients only at the beginning of the flying phase
    updatePolyCoeff_Z(pos_init, pos_end, t_swing_, maxHeight_ + height_);
    // Initale velocity and acceleration nulle
    updatePolyCoeff_XY(pos_init, Vector3::Zero(), Vector3::Zero(), pos_end, 0., t_swing_);

    // Update initial conditions of the Problem Definition
    pDef.init_pos = pos_init;
    pDef.init_vel = Vector3::Zero();
    pDef.init_acc = Vector3::Zero();
  }
  else {
      updatePolyCoeff_XY(pos_init, evaluatePoly(1,t0), evaluatePoly(2,t0),
                         pos_end, t0, t_swing_);
      // Update initial conditions of the Problem Definition
      pDef.init_pos = pos_init;
      pDef.init_vel = delta_t * evaluatePoly(1, t0);
      pDef.init_acc = std::pow(delta_t, 2) * evaluatePoly(2, t0);
    }
  // Update final conditions of the Problem Definition
  pDef.end_pos = pos_end;
  pDef.end_vel = Vector3::Zero();
  pDef.end_acc = Vector3::Zero();

  pDef.flag = optimization::INIT_POS | optimization::END_POS | optimization::INIT_VEL |
                        optimization::END_VEL | optimization::INIT_ACC | optimization::END_ACC;

  // generates the linear variable of the bezier curve with the parameters of problemDefinition
  problem_data_t pbData = optimization::setup_control_points<pointX_t, double, safe>(pDef);
  bezier_linear_variable_t* linear_bezier = pbData.bezier;

  P_.setZero();
  q_.setZero();
  linear_variable_t linear_var;
  double t_b_;
  for (int j = 0; j < N_samples; j++) {
    t_b_ = (j + 1.0) / N_samples;

    linear_var = linear_bezier->operator()(t_b_);

    P_ += linear_var.B().transpose() * linear_var.B();
    q_ += linear_var.B().transpose() * (linear_var.c() - evaluatePoly(0, t0 + (t_swing_ - t0) * t_b_));
  }

  // Eiquadprog-Fast solves the problem :
  // min. 1/2 * x' C_ x + q_' x
  // s.t. C_ x + d_ = 0
  //      G_ x + h_ >= 0
  status = qp.solve_quadprog(P_, q_, C_, d_, G_, h_, x);

  // Evaluate Bezier Linear with optimsed Points
  fitBezier = evaluateLinear<bezier_t, bezier_linear_variable_t>(*linear_bezier, x);

  // Reshape Bezier curve with proper boundaries T_min and T_max
  curve_ =  bezier_t::bezier_curve_t(fitBezier.waypoints().begin(), fitBezier.waypoints().end(), t0, t_swing_);
}

void FootTrajectoryBezier::update(Vector3 const& pos_init, Vector3 const& vel_init, Vector3 const& pos_end, double t0,
              Surface surface_init, Surface surface_end){
  double delta_t = t_swing_ - t0;
  t0_ = t0;
  if (t0 < 10e-4) {
    double height_ = std::max(pos_init(2), pos_end(2));
    // Update Z coefficients only at the beginning of the flying phase
    updatePolyCoeff_Z(pos_init, pos_end, t_swing_, maxHeight_ + height_);
    // Initale velocity and acceleration nulle
    updatePolyCoeff_XY(pos_init, Vector3::Zero(), Vector3::Zero(), pos_end, 0., t_swing_);

    // Update initial conditions of the Problem Definition
    // Update initial conditions of the Problem Definition
    Vector3 vector;
    vector << pos_init(0), pos_init(1), evaluatePoly(0,t0)(2);
    pDef.init_pos = vector;

    vector << 0., 0., delta_t * evaluatePoly(1, t0)(2);
    pDef.init_vel = vector;

    vector << 0., 0., std::pow(delta_t, 2) * evaluatePoly(2, t0)(2);
    pDef.init_acc = vector;
    // pDef.init_pos = pos_init;
    // pDef.init_vel = Vector3::Zero();
    // pDef.init_acc = Vector3::Zero();
    t_stop_ = 0.;
    x_margin_ = 0.;
    starting_position_ = pos_init;

    // Get inequality constraint form the new surface if going upward for now.
    if ((surface_end.getHeight(pos_end.head(2)) -
           starting_position_(2)) >= 0.02)
      {
        int nb_vert = surface_end.vertices_.rows();
        MatrixN vert = surface_end.vertices_;

        // Projection on X,Y axis of the segment pos_init vs pos_end to get
        // which vertices of the convex surface is crossed.
        Vector2 P1 = pos_init.head(2);
        Vector2 P2 = pos_end.head(2);

        Vector2 Q1;
        Vector2 Q2;
        for (int l = 0; l < nb_vert; l++) {
          Q1 << vert(l, 0), vert(l, 1);
          if (l < nb_vert - 1) {
            Q2 << vert(l + 1, 0), vert(l + 1, 1);
          } else {
            Q2 << vert(0, 0), vert(0, 1);
          }
          if (doIntersect_segment(P1, P2, Q1, Q2)) {
            get_intersect_segment(P1, P2, Q1, Q2);
            //  Should be sorted
            //  self.ineq = surface.ineq[k, :]
            //  self.ineq_vect = surface.ineq_vect[k]
            double a = 0.;
            if ((Q1[0] - Q2[0]) != 0.) {
              a = (Q2[1] - Q1[1]) / (Q2[0] - Q1[0]);
              double b = Q1[1] - a * Q1[0];
              ineq_ << -a, 1., 0.;  // -ax + y = b
              ineq_vector_ = b;
            } else {
              //  Inequality of the surface corresponding to these vertices
              ineq_ << -1., 0., 0.;
              ineq_vector_ = -Q1[0];
            }
            if (ineq_.transpose() * pos_init > ineq_vector_) {
              // Wrong side, the targeted point is inside the surface
              ineq_ = -ineq_;
              ineq_vector_ = -ineq_vector_;
            }
            // If foot position already closer than margin
            x_margin_ = std::max(std::min(x_margin_max_, std::abs(intersectionPoint_[0] - P1[0]) - 0.001), 0.);
          }
        }
      } else {
        ineq_.setZero();
        ineq_vector_ = 0.;
      }
  }
  else {
      updatePolyCoeff_XY(pos_init, evaluatePoly(1,t0), evaluatePoly(2,t0),
                         pos_end, t0, t_swing_);
      // Update initial conditions of the Problem Definition
      pDef.init_pos = pos_init;
      // pDef.init_vel = delta_t * evaluatePoly(1, t0);
      // pDef.init_acc = std::pow(delta_t, 2) * evaluatePoly(2, t0);
      pDef.init_vel = delta_t * curve_.derivate(t0,1);
      pDef.init_acc = std::pow(delta_t, 2) * curve_.derivate(t0, 2);
    }
  // Update final conditions of the Problem Definition
  pDef.end_pos = pos_end;
  pDef.end_vel = Vector3::Zero();
  pDef.end_acc = Vector3::Zero();

  pDef.flag = optimization::INIT_POS | optimization::END_POS | optimization::INIT_VEL |
                        optimization::END_VEL | optimization::INIT_ACC | optimization::END_ACC;

  // generates the linear variable of the bezier curve with the parameters of problemDefinition
  problem_data_t pbData = optimization::setup_control_points<pointX_t, double, safe>(pDef);
  bezier_linear_variable_t* linear_bezier = pbData.bezier;

  // Prepare the inequality matrix :
  Vector3 x_t = evaluatePoly(0, t0);
  double t_margin = t_margin_ * t_swing_;  // 10% around the limit point !inferior to 1/nb point in linspace

  // No surface switch or already overpass the critical point
  if (!ineq_.isZero()) {
    if (((x_t[2] < pos_end(2))) and x_margin_ != 0. or (t0_ < t_stop_ + t_margin)) {
      int nb_vert = surface_end.vertices_.rows();
      MatrixN vert = surface_end.vertices_;

      // Projection on X,Y axis of the segment pos_init vs pos_end to get
      // which vertices of the convex surface is crossed.
      Vector2 P1 = pos_init.head(2);
      Vector2 P2 = pos_end.head(2);

      Vector2 Q1;
      Vector2 Q2;
      for (int l = 0; l < nb_vert; l++) {
        Q1 << vert(l, 0), vert(l, 1);
        if (l < nb_vert - 1) {
          Q2 << vert(l + 1, 0), vert(l + 1, 1);
        } else {
          Q2 << vert(0, 0), vert(0, 1);
        }
        if (doIntersect_segment(P1, P2, Q1, Q2)) {
          get_intersect_segment(P1, P2, Q1, Q2);
          //  Should be sorted
          //  self.ineq = surface.ineq[k, :]
          //  self.ineq_vect = surface.ineq_vect[k]
          double a = 0.;
          if ((Q1[0] - Q2[0]) != 0.) {
            a = (Q2[1] - Q1[1]) / (Q2[0] - Q1[0]);
            double b = Q1[1] - a * Q1[0];
            ineq_ << -a, 1., 0.;  // -ax + y = b
            ineq_vector_ = b;
          } else {
            //  Inequality of the surface corresponding to these vertices
            ineq_ << -1., 0., 0.;
            ineq_vector_ = -Q1[0];
          }
          if (ineq_.transpose() * pos_init > ineq_vector_) {
            // Wrong side, the targeted point is inside the surface
            ineq_ = -ineq_;
            ineq_vector_ = -ineq_vector_;
          }
          // If foot position already closer than margin
          x_margin_ = std::max(std::min(x_margin_max_, std::abs(intersectionPoint_[0] - P1[0]) - 0.001), 0.);
        }
      }

      double z_margin = pos_end(2) * z_margin_;  // 10% around the limit height
      double t_s;
      double zt;

      linear_variable_t linear_var_;

      for (int its = 0; its < N_samples_ineq; its++) {
        t_s = (its + 1.0) / N_samples_ineq;
        zt = evaluatePoly(0, t0 + (t_swing_ - t0) * t_s)[2];
        if (t0 + (t_swing_ - t0) * t_s < t_stop_ + t_margin) {
          if (zt < pos_end(2) + z_margin) {
            t_stop_ = t0 + (t_swing_ - t0) * t_s;
          }
          linear_var_ = linear_bezier->operator()(t_s);
          G_.row(its) = -ineq_.transpose() * linear_var_.B();
          h_(its) = -ineq_.transpose() * linear_var_.c() + ineq_vector_ - (x_margin_);
        } else {
          G_.row(its).setZero();
          h_(its) = 0.;
        }
      }
    }
  } else {
    G_.setZero();
    for (int l = 0; l < h_.size(); l++) {
      h_(l) = 0.;
    }
  }

  P_.setZero();
  q_.setZero();
  linear_variable_t linear_var;
  double t_b_;
  for (int j = 0; j < N_samples; j++) {
    t_b_ = (j + 1.0) / N_samples;

    linear_var = linear_bezier->operator()(t_b_);

    P_ += linear_var.B().transpose() * linear_var.B();
    q_ += linear_var.B().transpose() * (linear_var.c() - evaluatePoly(0, t0 + (t_swing_ - t0) * t_b_));
  }

  // Eiquadprog-Fast solves the problem :
  // min. 1/2 * x' C_ x + q_' x
  // s.t. C_ x + d_ = 0
  //      G_ x + h_ >= 0
  status = qp.solve_quadprog(P_, q_, C_, d_, G_, h_, x);

  // Evaluate Bezier Linear with optimsed Points
  fitBezier = evaluateLinear<bezier_t, bezier_linear_variable_t>(*linear_bezier, x);

  // Reshape Bezier curve with proper boundaries T_min and T_max
  curve_ =  bezier_t::bezier_curve_t(fitBezier.waypoints().begin(), fitBezier.waypoints().end(), t0, t_swing_);
}

Vector3 FootTrajectoryBezier::evaluateBezier(int const& indice, double const& t) {
  if (indice == 0) {
    return curve_(t);
  } else {
    return curve_.derivate(t, indice);
  }
}

Vector3 FootTrajectoryBezier::evaluatePoly(int const& indice, double const& t) {
  Vector3 vector = Vector3::Zero();
  if (indice == 0) {
    double x = Ax(0) + Ax(1) * t + Ax(2) * std::pow(t, 2) + Ax(3) * std::pow(t, 3) +
               Ax(4) * std::pow(t, 4) + Ax(5) * std::pow(t, 5);
    double y = Ay(0) + Ay(1) * t + Ay(2) * std::pow(t, 2) + Ay(3) * std::pow(t, 3) +
               Ay(4) * std::pow(t, 4) + Ay(5) * std::pow(t, 5);
    double z = Az(0) + Az(1) * t + Az(2) * std::pow(t, 2) + Az(3) * std::pow(t, 3) +
               Az(4) * std::pow(t, 4) + Az(5) * std::pow(t, 5) + Az(6) * std::pow(t, 6);
    vector << x, y, z;
  }

  if (indice == 1) {
    double vx = Ax(1) + 2 * Ax(2) * t + 3 * Ax(3) * std::pow(t, 2) +
                4 * Ax(4) * std::pow(t, 3) + 5 * Ax(5) * std::pow(t, 4);
    double vy = Ay(1) + 2 * Ay(2) * t + 3 * Ay(3) * std::pow(t, 2) +
                4 * Ay(4) * std::pow(t, 3) + 5 * Ay(5) * std::pow(t, 4);
    double vz = Az(1) + 2 * Az(2) * t + 3 * Az(3) * std::pow(t, 2) +
                4 * Az(4) * std::pow(t, 3) + 5 * Az(5) * std::pow(t, 4) +
                6 * Az(6) * std::pow(t, 5);
    vector << vx, vy, vz;
  }

  if (indice == 2) {
    double ax = 2 * Ax(2) + 6 * Ax(3) * t + 12 * Ax(4) * std::pow(t, 2) +
                20 * Ax(5) * std::pow(t, 3);
    double ay = 2 * Ay(2) + 6 * Ay(3) * t + 12 * Ay(4) * std::pow(t, 2) +
                20 * Ay(5) * std::pow(t, 3);
    double az = 2 * Az(2) + 6 * Az(3) * t + 12 * Az(4) * std::pow(t, 2) +
                20 * Az(5) * std::pow(t, 3) + 30 * Az(6) * std::pow(t, 4);
    vector << ax, ay, az;
  }

  return vector;
}

void FootTrajectoryBezier::updatePolyCoeff_XY(Vector3 const& x_init, Vector3 const& v_init, Vector3 const& a_init,
                                              Vector3 const& x_target, double const& t0, double const& t1) {
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

void FootTrajectoryBezier::updatePolyCoeff_Z(Vector3 const& x_init, Vector3 const& x_target, double const& t1,
                                             double const& h) {
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

bool FootTrajectoryBezier::doIntersect_segment(Vector2 const& p1, Vector2 const& q1, Vector2 const& p2,
                                               Vector2 const& q2) {
  //  Find the 4 orientations required for
  //  the general and special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  //  General case
  if ((o1 != o2) and (o3 != o4)) {
    return true;
  }

  //  Special Cases
  //  p1 , q1 and p2 are colinear and p2 lies on segment p1q1
  if ((o1 == 0) and onSegment(p1, p2, q1)) {
    return true;
  }

  //  p1 , q1 and q2 are colinear and q2 lies on segment p1q1
  if ((o2 == 0) and onSegment(p1, q2, q1)) {
    return true;
  }

  //  p2 , q2 and p1 are colinear and p1 lies on segment p2q2
  if ((o3 == 0) and onSegment(p2, p1, q2)) {
    return true;
  }

  //  p2 , q2 and q1 are colinear and q1 lies on segment p2q2
  if ((o4 == 0) and onSegment(p2, q1, q2)) {
    return true;
  }

  //  If none of the cases
  return false;
}

//  Given three colinear points p, q, r, the function checks if
//  point q lies on line segment 'pr'
bool FootTrajectoryBezier::onSegment(Vector2 const& p, Vector2 const& q, Vector2 const& r) {
  if ((q[0] <= std::max(p[0], r[0])) and (q[0] >= std::min(p[0], r[0])) and (q[1] <= std::max(p[1], r[1])) and
      (q[1] >= std::min(p[1], r[1]))) {
    return true;
  }
  return false;
}

// to find the orientation of an ordered triplet (p,q,r)
// function returns the following values:
// 0 : Colinear points
// 1 : Clockwise points
// 2 : Counterclockwise
// See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
// for details of below formula.
// Modified, remove class Point, directly handle p = [px,py]
int FootTrajectoryBezier::orientation(Vector2 const& p, Vector2 const& q, Vector2 const& r) {
  double val = ((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]));
  if (val > 0) {
    // Clockwise orientation
    return 1;
  } else if (val < 0) {
    // Counterclockwise orientation
    return 2;
  } else {
    // Colinear orientation
    return 0;
  }
}

//  Method to intersect 2 segment --> useful to retrieve which inequality is crossed in a surface (2D)
// Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
// a1: [x, y] a point on the first line
// a2: [x, y] another point on the first line
// b1: [x, y] a point on the second line
// b2: [x, y] another point on the second line

void FootTrajectoryBezier::get_intersect_segment(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2) {
  Vector3 cross_l1;
  Vector3 cross_l2;
  Vector3 cross_ll;

  cross_l1 << a1[1] - a2[1], a1[0] - a2[0], a2[0] * a1[1] - a2[1] * a1[0];
  cross_l2 << b1[1] - b2[1], b1[0] - b2[0], b2[0] * b1[1] - b2[1] * b1[0];

  cross_ll << cross_l1[1] - cross_l2[1], cross_l1[0] - cross_l2[0],
      cross_l2[0] * cross_l1[1] - cross_l2[1] * cross_l1[0];

  intersectionPoint_(0) = cross_ll[0] / cross_ll[2];
  intersectionPoint_(1) = cross_ll[1] / cross_ll[2];
}
