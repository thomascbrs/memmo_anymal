#include <Gait.hpp>

QuadrupedalGaitGenerator::~QuadrupedalGaitGenerator(){};

QuadrupedalGaitGenerator::QuadrupedalGaitGenerator(double dt, int S,
                                                   std::string lf,
                                                   std::string lh,
                                                   std::string rf,
                                                   std::string rh)
    : dt_(dt), S_(S), lf_(lf), lh_(lh), rf_(rf), rh_(rh) {
  contactNames_.push_back(lf_);
  contactNames_.push_back(lh_);
  contactNames_.push_back(rf_);
  contactNames_.push_back(rh_);
};

std::shared_ptr<ContactSchedule>
QuadrupedalGaitGenerator::walk(StdVec_Map_string_SE3 contacts, int N_ds,
                               int N_ss, int N_uds = 0, int N_uss = 0,
                               double stepHeight = 0.15, bool startPhase = true,
                               bool endPhase = true) {
  if (contacts.size() != 2) {
    throw std::runtime_error(
        "The size of the contact should be 2 (starting and ending position0");
  }
  try {
    // Check if contact[0][lf_] and contact[1][lf_] ..etc does exits.
    for (size_t i = 0; i < 2; i++) {
      for (const auto &name : contactNames_) {
        contacts[i].at(name);
      }
    }
  } catch (std::out_of_range &e) {
    throw std::runtime_error(
        "Key names in the contact dictionary are not consistent with the ones "
        "defined in initial constructor.");
  }

  int N_0 = 0;
  if (startPhase) {
    N_0 = N_ds;
  }
  int N;
  if (endPhase) {
    N = N_0 + 2 * N_ds + 4 * N_ss - 2 * N_uss - N_uds;
  } else {
    N = N_0 + N_ds + 4 * N_ss - 2 * N_uss - N_uds;
  }
  std::shared_ptr<ContactSchedule> gait =
      std::make_shared<ContactSchedule>(dt_, N, S_, contactNames_);
  std::shared_ptr<FootTrajectoryWrapper> lfSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][lf_], contacts[1][lf_]);
  std::shared_ptr<FootTrajectoryWrapper> lhSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][lh_], contacts[1][lh_]);
  std::shared_ptr<FootTrajectoryWrapper> rfSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][rf_], contacts[1][rf_]);
  std::shared_ptr<FootTrajectoryWrapper> rhSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][rh_], contacts[1][rh_]);

  std::vector<std::shared_ptr<ContactPhase>> lh_schedule = {
      std::make_shared<ContactPhase>(N_0),
      std::make_shared<ContactPhase>(N_ss, lhSwingTraj),
      std::make_shared<ContactPhase>(N - (N_0 + N_ss))};
  std::vector<std::shared_ptr<ContactPhase>> lf_schedule = {
      std::make_shared<ContactPhase>(N_0 + N_ss - N_uss),
      std::make_shared<ContactPhase>(N_ss, lfSwingTraj),
      std::make_shared<ContactPhase>(N - (N_0 + 2 * N_ss - N_uss))};
  std::vector<std::shared_ptr<ContactPhase>> rh_schedule = {
      std::make_shared<ContactPhase>(N_0 + N_ds + 2 * N_ss - N_uss - N_uds),
      std::make_shared<ContactPhase>(N_ss, rhSwingTraj),
      std::make_shared<ContactPhase>(N -
                                     (N_0 + N_ds + 3 * N_ss - N_uss - N_uds))};
  std::vector<std::shared_ptr<ContactPhase>> rf_schedule = {
      std::make_shared<ContactPhase>(N_0 + N_ds + 3 * N_ss - 2 * N_uss - N_uds),
      std::make_shared<ContactPhase>(N_ss, rfSwingTraj),
      std::make_shared<ContactPhase>(
          N - (N_0 + N_ds + 4 * N_ss - 2 * N_uss - N_uds))};

  gait->addSchedule(lh_, lh_schedule);
  gait->addSchedule(lf_, lf_schedule);
  gait->addSchedule(rh_, rh_schedule);
  gait->addSchedule(rf_, rf_schedule);

  return gait;
};

std::shared_ptr<ContactSchedule>
QuadrupedalGaitGenerator::trot(StdVec_Map_string_SE3 contacts, int N_ds,
                               int N_ss, int N_uds = 0, int N_uss = 0,
                               double stepHeight = 0.15, bool startPhase = true,
                               bool endPhase = true) {
  if (contacts.size() != 2) {
    throw std::runtime_error(
        "The size of the contact should be 2 (starting and ending position0");
  }
  try {
    // Check if contact[0][lf_] and contact[1][lf_] ..etc does exits.
    for (size_t i = 0; i < 2; i++) {
      for (const auto &name : contactNames_) {
        contacts[i].at(name);
      }
    }
  } catch (std::out_of_range &e) {
    throw std::runtime_error(
        "Key names in the contact dictionary are not consistent with the ones "
        "defined in initial constructor.");
  }

  int N_0 = 0;
  if (startPhase) {
    N_0 = N_ds;
  }
  int N;
  if (endPhase) {
    N = N_0 + N_ds + 2 * N_ss - 2 * N_uss;
  } else {
    N = N_0 + 2 * N_ss - N_uss;
  }
  std::shared_ptr<ContactSchedule> gait =
      std::make_shared<ContactSchedule>(dt_, N, S_, contactNames_);
  std::shared_ptr<FootTrajectoryWrapper> lfSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][lf_], contacts[1][lf_]);
  std::shared_ptr<FootTrajectoryWrapper> lhSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][lh_], contacts[1][lh_]);
  std::shared_ptr<FootTrajectoryWrapper> rfSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][rf_], contacts[1][rf_]);
  std::shared_ptr<FootTrajectoryWrapper> rhSwingTraj =
      std::make_shared<FootTrajectoryWrapper>(
          dt_, N_ss, stepHeight, contacts[0][rh_], contacts[1][rh_]);

  std::vector<std::shared_ptr<ContactPhase>> lh_schedule = {
      std::make_shared<ContactPhase>(N_0),
      std::make_shared<ContactPhase>(N_ss, lhSwingTraj),
      std::make_shared<ContactPhase>(N - (N_0 + N_ss))};
  std::vector<std::shared_ptr<ContactPhase>> rf_schedule = {
      std::make_shared<ContactPhase>(N_0),
      std::make_shared<ContactPhase>(N_ss, rfSwingTraj),
      std::make_shared<ContactPhase>(N - (N_0 + N_ss))};
  std::vector<std::shared_ptr<ContactPhase>> lf_schedule = {
      std::make_shared<ContactPhase>(N_0 + N_ss - N_uss),
      std::make_shared<ContactPhase>(N_ss, lfSwingTraj),
      std::make_shared<ContactPhase>(N - (N_0 + 2 * N_ss - N_uss))};
  std::vector<std::shared_ptr<ContactPhase>> rh_schedule = {
      std::make_shared<ContactPhase>(N_0 + N_ss - N_uss),
      std::make_shared<ContactPhase>(N_ss, rhSwingTraj),
      std::make_shared<ContactPhase>(N - (N_0 + 2 * N_ss - N_uss))};

  gait->addSchedule(lh_, lh_schedule);
  gait->addSchedule(rf_, rf_schedule);
  gait->addSchedule(lf_, lf_schedule);
  gait->addSchedule(rh_, rh_schedule);

  return gait;
};
