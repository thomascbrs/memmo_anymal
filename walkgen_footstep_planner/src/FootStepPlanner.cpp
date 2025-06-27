#include "FootStepPlanner.hpp"

#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/se3.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

FootStepPlanner::FootStepPlanner(const pinocchio::Model &model, const Eigen::VectorXd &q, const Params &params,
                                 const double period)
    : params_(params), model_(model), period_(period) {
  data_ = pinocchio::Data(model_);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  filter_q = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
  filter_v = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
  initialize(q);
}

FootStepPlanner::FootStepPlanner(const pinocchio::Model &model, const Eigen::VectorXd &q, const double period)
    : params_(Params()), model_(model), period_(period) {
  data_ = pinocchio::Data(model_);
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  filter_q = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
  filter_v = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
  initialize(q);
}

void FootStepPlanner::initialize(const Eigen::VectorXd &q) {
  // const std::string lf = params_.feet_names[0];
  // const std::string lh = params_.feet_names[2];
  // const std::string rf = params_.feet_names[1];
  // const std::string rh = params_.feet_names[3];

  contactNames_ = params_.feet_names;
  contactNames_sl1m_ = params_.feet_names_sl1m;

  // TODO find a better way for this
  // offsets_feet_[contactNames_[0]] = Vector3(0.367, 0.2, 0.);
  // offsets_feet_[contactNames_[1]] = Vector3(0.367, -0.2, 0.);
  // offsets_feet_[contactNames_[2]] = Vector3(-0.367, 0.2, 0.);
  // offsets_feet_[contactNames_[3]] = Vector3(-0.367, -0.2, 0.);

  if (contactNames_.size() != params_.shoulder_offsets.size()) {
    throw std::runtime_error(
        "The contact name list and the shoulder offsets should "
        "have the same size.");
  }
  if (contactNames_sl1m_.size() != contactNames_.size()) {
    throw std::runtime_error(
        "The size of each offset in shoulder offset params "
        "should be size 2 (x and y axis).");
  }
  for (auto elem : params_.shoulder_offsets) {
    if (elem.size() != 2) {
      throw std::runtime_error(
          "The size of each offset in shoulder offset params "
          "should be size 2 (x and y axis).");
    }
  }
  for (size_t j = 0; j < contactNames_.size(); j++) {
    Vector3 offset = Vector3::Zero();
    offset[0] = params_.shoulder_offsets[j][0];
    offset[1] = params_.shoulder_offsets[j][1];
    offsets_feet_[contactNames_[j]] = offset;
  }
  std::sort(contactNames_.begin(), contactNames_.end());

  current_position_ = MatrixN::Zero(3, 4);
  current_velocities_ = Matrix34::Zero();
  target_fsteps_ = Matrix34::Zero();
  qf_ = VectorN::Zero(18);
  qvf_ = VectorN::Zero(18);

  nsteps_ = params_.nsteps;
  horizon_ = params_.horizon;
  dt_ = params_.dt;
  counter_gait_ = 0;
  early_termination_ratio_ = params_.early_termination_ratio;

  double dx = 0.5;
  double dy = 0.5;
  double height = 0.;
  double epsilon = 10e-6;
  Eigen::Matrix<double, 6, 3> A;
  A << -1., 0., 0., 0., -1., 0., 0., 1., 0., 1., 0., 0., 0., 0., 1., 0., 0., -1.;
  Eigen::Matrix<double, 6, 1> b;
  b << dx - q(0), dy - q(1), dy + q(1), dx + q(0), height + epsilon, -height + epsilon;

  Eigen::Matrix<double, 4, 3> vertices;
  vertices << q[0] - dx, q[1] + dy, height, q[0] - dx, q[1] - dy, height, q[0] + dx, q[1] - dy, height, q[0] + dx,
      q[1] + dy, height;
  for (size_t foot = 0; foot < 4; ++foot) {
    previous_surfaces_[contactNames_[foot]] = Surface(MatrixN(A), b, MatrixN(vertices));
  }

  // Initialize the tmp variables
  oMf_tmp = pinocchio::SE3::Identity();
  v_tmp = pinocchio::Motion::Zero();
  footstep_tmp.setZero();
  cross_tmp.setZero();
  q_filter_tmp.setZero();
  Rz.setZero();
  Rxy.setZero();
  q_tmp.setZero();
  P0.setZero();
  V0.setZero();
  cs_index = 0;
  timeline = 0;
  foot_timeline = {0, 0, 0, 0};
  target_fsteps = MatrixN::Zero(3, 4);
  Rz_tmp = Matrix3::Identity();
  dt_i = 0.;
  dx = 0.;
  dy = 0.;
  q_dxdy.setZero();
  C_ = MatrixN::Zero(3, 0);
  d_ = VectorN::Zero(0);
  delta_x = VectorN::Zero(3);
  G_ = MatrixN::Zero(3, 3);  // Random size
  h_ = VectorN::Zero(3);
  fsteps_optim.setZero();
  previous_sf = Surface(MatrixN(A), b, MatrixN(vertices));
};

// From python, it is easier to use and return only MatrixN, otherwise, a
// binding is necessary for each type.
MatrixN FootStepPlanner::compute_footstep(std::vector<std::shared_ptr<ContactSchedule>> &queue_cs, const VectorN &q,
                                          const VectorN &vq, const VectorN &bvref, const int timeline,
                                          const std::map<std::string, std::vector<Surface>> &selected_surfaces,
                                          const std::map<std::string, Surface> &previous_surfaces) {
  if (q.size() != 19) {
    throw std::runtime_error(
        "Current state q should be an array of size 19 "
        "[pos (x3), quaternion (x4), joint (x12)]");
  }
  if (vq.size() != 18) {
    throw std::runtime_error(
        "Current velocity vq should be an array of size "
        "18 [lin vel (x3), ang vel (x3), joint vel(x12)]");
  }
  if (bvref.size() != 6) {
    throw std::runtime_error(
        "Reference velocity should be an array of size 6 "
        "[lin vel (x3), ang vel (x3)]");
  }

  // Update current feet position
  update_current_state(q, vq);

  // Filter quantities
  q_filter_tmp.head<3>() = q.head<3>();
  q_filter_tmp.tail<3>() =
      pinocchio::rpy::matrixToRpy(pinocchio::SE3::Quaternion(q(6), q(3), q(4), q(5)).toRotationMatrix());

  qf_ = filter_q.filter(q_filter_tmp);
  qvf_ = filter_v.filter(vq.head<6>());

  return update_position(queue_cs, qf_, qvf_, bvref, timeline, selected_surfaces);
}

MatrixN FootStepPlanner::update_position(std::vector<std::shared_ptr<ContactSchedule>> &queue_cs, const Vector6 &q,
                                         const Vector6 &vq, const VectorN &bvref, const int timeline_in,
                                         const std::map<std::string, std::vector<Surface>> &selected_surfaces) {
  if (timeline_in == 0) {
    counter_gait_ += 1;
  }
  Rz = pinocchio::rpy::rpyToMatrix(double(0.), double(0.), q(5));
  Rxy = pinocchio::rpy::rpyToMatrix(q(3), q(4), double(0.));

  q_tmp = q.head<3>();
  q_tmp(2) = double(0.);

  P0 = current_position_;
  V0 = current_velocities_;
  timeline = timeline_in;

  // Reset quantities
  cs_index = 0;
  std::fill(foot_timeline.begin(), foot_timeline.end(), 0);
  target_fsteps.setZero();
  Rz_tmp.setIdentity();

  for (auto cs_iter = queue_cs.rbegin(); cs_iter != queue_cs.rend(); ++cs_iter) {
    auto &cs = **cs_iter;  // Reference to ContactSchedule
    if (cs_index <= horizon_ + 2) {
      for (size_t c = 0; c < cs.contactNames_.size(); ++c) {
        auto &name = cs.contactNames_[c];
        size_t j = find_stdVec(cs.contactNames_, name);  // Which foot in foot_timeline
        auto &phases = cs.phases_[c];
        if (phases.size() == 3) {
          double T_stance = double(phases[0]->T_ + phases[2]->T_) * cs.dt_;
          auto &active_phase = phases[0];
          auto &inactive_phase = phases[1];

          if (cs_index + active_phase->T_ - timeline <= horizon_ + 2) {
            // Displacement following the reference velocity compared to current
            // position
            if (active_phase->T_ + inactive_phase->T_ - timeline > 0) {  // case 1 and 2
              if (std::abs(bvref(5)) > 0.01) {
                dt_i = static_cast<double>(cs_index + active_phase->T_ + inactive_phase->T_ - timeline) *
                       cs.dt_;  // dt integration dt_i
                dx = (bvref(0) * std::sin(bvref(5) * dt_i) + bvref(1) * (std::cos(bvref(5) * dt_i) - 1.0)) / bvref(5);
                dy = (bvref(1) * std::sin(bvref(5) * dt_i) - bvref(0) * (std::cos(bvref(5) * dt_i) - 1.0)) / bvref(5);
                dt_i = static_cast<double>(cs_index + active_phase->T_ + inactive_phase->T_) * cs.dt_;
                Rz_tmp = pinocchio::rpy::rpyToMatrix(double(0.), double(0.), bvref(5) * dt_i);
              } else {
                dt_i = static_cast<double>(cs_index + active_phase->T_ + inactive_phase->T_ - timeline) * cs.dt_;
                dx = bvref(0) * dt_i;
                dy = bvref(1) * dt_i;
                Rz_tmp = Matrix3::Identity();
              }
              q_dxdy << dx, dy, double(0.);
              heuristic_tmp = compute_heuristic(vq, bvref, Rxy, T_stance, name, false);
              footstep_ref = Rz * (Rz_tmp * heuristic_tmp) + q_tmp + Rz * q_dxdy;

              Matrix3 P_ = Matrix3::Identity();
              Vector3 q_ = Vector3::Zero();
              auto iter = selected_surfaces.find(name);
              if (iter == selected_surfaces.end()) {
                throw std::runtime_error("Naming non consistent in surface dictionnary.");
              }
              auto &sf_ = (iter->second).at(foot_timeline.at(j));
              G_ = -sf_.getA();                             // Inverse sign in python
              h_ = sf_.getb() - sf_.getA() * footstep_ref;  // Inverse sign in python

              delta_x.setZero();
              // In python code
              // delta_x = quadprog_solve_qp(P_, q_, G_, h_)
              // min (1/2)x' P x + q' x
              // subject to  G x <= h
              // subject to  C x  = d

              // Eiquadprog-Fast solves the problem :
              // min. 1/2 * x' P_ x + q_' x
              // s.t. C_ x + d_ = 0
              //      G_ x + h_ >= 0
              status = qp.solve_quadprog(P_, q_, C_, d_, G_, h_, delta_x);
              fsteps_optim = footstep_ref + delta_x;

              // Update target fsteps for sl1m
              if (foot_timeline.at(j) == 0) {
                size_t jj = find_stdVec(contactNames_sl1m_, name);
                target_fsteps_.col(static_cast<Eigen::Index>(jj)) = fsteps_optim;
              }

              // Update previous surface mechanism
              if (foot_timeline.at(j) == 0) {
                auto iter = previous_surfaces_.find(name);
                if (iter == previous_surfaces_.end()) {
                  throw std::runtime_error("Naming non consistent in previous_surface dictionnary.");
                }
                previous_sf = Surface(iter->second);  // make a copy
              } else {
                auto iter = selected_surfaces.find(name);
                if (iter == selected_surfaces.end()) {
                  throw std::runtime_error("Naming non consistent in selected_surface dictionnary.");
                }
                previous_sf = Surface((iter->second).at(foot_timeline.at(j) - 1));
              }
              double t0 = double(0.);

              // Update trajectory
              if (active_phase->T_ - timeline >= 0) {
                V0.col(static_cast<Eigen::Index>(j)).setZero();
              } else {
                t0 = static_cast<double>(timeline - active_phase->T_);
              }
              if (counter_gait_ < 2) {
                // Letting 1/2 gait for the estimator at first since it is a
                // moving average estimator. Hence, keeping initial position.
                fsteps_optim.head<2>() = P0.col(static_cast<Eigen::Index>(j)).head<2>();
              }
              if (t0 <= static_cast<double>(inactive_phase->T_) * early_termination_ratio_) {
                inactive_phase->trajectory_->update(P0.col(static_cast<Eigen::Index>(j)),
                                                    V0.col(static_cast<Eigen::Index>(j)), fsteps_optim, t0 * cs.dt_,
                                                    sf_, previous_sf);
              }

              // End of the flying phase, register the surface.
              if (t0 >= static_cast<double>(inactive_phase->T_ - nsteps_)) {
                // previous_surfaces_ .pop(name)
                auto iter = previous_surfaces_.find(name);
                if (iter == previous_surfaces_.end()) {
                  throw std::runtime_error("Naming non consistent in previous_surface dictionnary.");
                }
                iter->second = Surface(sf_);  // make a copy.
              }
              P0.col(static_cast<Eigen::Index>(j)) = fsteps_optim;
              V0.col(static_cast<Eigen::Index>(j)).setZero();

              foot_timeline.at(j) += 1;
            } else {  // case 3
              V0.col(static_cast<Eigen::Index>(j)).setZero();
            }
          }
        } else {
          throw std::runtime_error(
              "Error in fstep planner. Only 3 phases per "
              "ContactSchedule considered.");
        }
      }
    } else {
      break;
    }
    cs_index += cs.T_ - timeline;
    timeline = 0;
  }

  return target_fsteps_;
}

void FootStepPlanner::update_current_state(const Eigen::VectorXd &q, const Eigen::VectorXd &vq) {
  if (q.size() != 19) {
    throw std::runtime_error(
        "Current state q should be an array of size 19 "
        "[pos (x3), quaternion (x4), joint (x12)]");
  }
  if (vq.size() != 18) {
    throw std::runtime_error(
        "Current velocity vq should be an array of size "
        "18 [lin vel (x3), ang vel (x3), joint vel(x12)]");
  }
  pinocchio::forwardKinematics(model_, data_, q, vq);
  size_t frame_id;
  for (size_t i = 0; i < contactNames_.size(); i++) {
    frame_id = model_.getFrameId(contactNames_[i]);
    oMf_tmp = pinocchio::updateFramePlacement(model_, data_, frame_id);
    v_tmp = pinocchio::getFrameVelocity(model_, data_, frame_id);
    current_position_.col(static_cast<Eigen::Index>(i)) = oMf_tmp.translation();
    current_velocities_.col(static_cast<Eigen::Index>(i)) = v_tmp.linear();
  }
}

Vector3 FootStepPlanner::compute_heuristic(const Eigen::VectorXd &bv, const Eigen::VectorXd &bvref, const Matrix3 &Rxy,
                                           const double T_stance, const std::string &name, const bool feedback_term) {
  // Reset tmp variables.
  footstep_tmp.setZero();
  cross_tmp.setZero();

  // Add symmetry term
  footstep_tmp += beta_ * T_stance * bvref.head<3>();

  // Add feedback term
  if (feedback_term) {
    footstep_tmp += k_feedback_ * bv.head<3>();
    footstep_tmp += -k_feedback_ * bvref.head<3>();
  }

  // Add centrifugal term
  Vector3 cross;
  cross_tmp << bv(1) * bvref(5) - bv(2) * bvref(4), bv(2) * bvref(3) - bv(0) * bvref(5), 0.0;
  footstep_tmp += 0.5 * std::sqrt(href_ / g_) * cross_tmp;

  // Limit deviation
  footstep_tmp(0) = std::min(footstep_tmp(0), L_);
  footstep_tmp(0) = std::max(footstep_tmp(0), -L_);
  footstep_tmp(1) = std::min(footstep_tmp(1), L_);
  footstep_tmp(1) = std::max(footstep_tmp(1), -L_);

  // Add shoulders, Yaw axis taken into account later
  // size_t j = find_stdVec(contactNames_,name);
  footstep_tmp += Rxy * offsets_feet_.at(name);

  // Remove Z component (working on flat ground)
  footstep_tmp(2) = 0.0;

  return footstep_tmp;
}

// find element inside vector list.
template <typename Type>
size_t FootStepPlanner::find_stdVec(const std::vector<Type> &vec, const Type &elem) {
  auto it = std::find(vec.begin(), vec.end(), elem);
  if (it != vec.end()) {
    return static_cast<size_t>(std::distance(vec.begin(), it));
  } else {
    throw std::runtime_error("Element not found in vector");
  }
}
