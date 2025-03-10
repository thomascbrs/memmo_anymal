#ifndef GAITMANAGER_HPP
#define GAITMANAGER_HPP

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include <vector>

#include "ContactPhase.hpp"
#include "ContactSchedule.hpp"
#include "Gait.hpp"
#include "Params.hpp"

// C++ struct for the Bezier coefficients.
struct CoeffBezier {
  double t0;
  Eigen::MatrixXd coeffs;

  bool operator==(const CoeffBezier &other) const { return t0 == other.t0 && coeffs == other.coeffs; }

  bool operator!=(const CoeffBezier &other) const { return t0 != other.t0 or coeffs != other.coeffs; }

  // Constructor
  CoeffBezier(double t0_, Eigen::MatrixXd coeffs_) : t0(t0_), coeffs(coeffs_) {}

  // Accessors
  double get_t0() const { return t0; }
  Eigen::MatrixXd get_coeffs() const { return coeffs; }
};

class GaitManager {
 public:
  GaitManager(const pinocchio::Model &model, const VectorN &q);
  GaitManager(const pinocchio::Model &model, const VectorN &q, const Params &params);
  void initialize(const pinocchio::Model &model, const VectorN &q);
  void print_queue();
  void print_current_switches() { print_switches(current_switches_); };
  bool update();
  MatrixN_int get_gait() { return current_gait_; };
  double get_timeline() { return timeline_; };
  bool is_new_step() { return new_step_; };
  std::vector<std::vector<std::vector<CoeffBezier>>> get_coefficients();
  std::map<std::string, pinocchio::SE3> cs0;
  std::map<std::string, pinocchio::SE3> cs1;
  std::vector<std::shared_ptr<ContactSchedule>> queue_cs_;
  std::vector<std::shared_ptr<ContactSchedule>> get_cs() { return queue_cs_; };
  std::vector<double> get_timings() { return timings; };
  void set_next_gait(const int cmd);
  int get_next_gait() { return cmd_gait_; };

 private:
  std::vector<int> evaluate_config(const ContactSchedule &schedule, int timeline);
  void update_switches(const std::vector<std::shared_ptr<ContactSchedule>> cs_queue,
                       std::map<int, std::vector<int>> &switches, const int timeline_in, bool use_next_cs = true);
  void print_switches(std::map<int, std::vector<int>> &switches);
  MatrixN_int compute_gait(int timeline);
  std::shared_ptr<ContactSchedule> get_next_cs();
  std::map<int, std::vector<int>> get_next_switch();

  Params params_;

  std::shared_ptr<ContactSchedule> initial_schedule_;
  std::shared_ptr<ContactSchedule> default_schedule_;
  std::shared_ptr<ContactSchedule> walk_schedule_;
  std::shared_ptr<ContactSchedule> trot_schedule_;

  std::vector<std::string> contactNames_;       // Contact names list.
  std::vector<std::string> contactNames_sl1m_;  // Contact names list SL1M.

  std::string type_;
  double dt_;
  int N_ss_;
  int N_ds_;
  int N_uds_;
  int N_uss_;
  int nsteps_;
  double stepHeight_;
  int N_phase_return_;

  int horizon_;
  int timeline_;
  bool new_step_;
  bool is_first_gait_;
  MatrixN_int current_gait_;
  int ratio_nsteps_;
  int new_step_counter_;
  std::map<int, std::vector<int>> current_switches_;
  std::map<int, std::vector<int>> walk_switches_;
  std::map<int, std::vector<int>> trot_switches_;
  std::map<int, std::vector<int>> default_switches_;
  int NGAIT;
  std::vector<double> timings;
  // std::map<int, std::vector<int>> switches_tmp_;
  int cmd_gait_;  // Next cmd gait
};

#endif  // GAITMANAGER_HPP
