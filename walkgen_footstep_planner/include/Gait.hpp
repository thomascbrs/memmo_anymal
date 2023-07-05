#ifndef GAIT_HPP
#define GAIT_HPP

#include <ContactSchedule.hpp>
#include <FootTrajectoryWrapper.hpp>
#include <string>
#include <vector>

// typedef std::map<std::string, pinocchio::SE3, std::less<std::string>,
// std::allocator<std::pair<const std::string,pinocchio::SE3>> >
// StdMap_string_SE3;
typedef std::map<std::string, pinocchio::SE3> StdMap_string_SE3;
typedef std::vector<StdMap_string_SE3> StdVec_Map_string_SE3;

class QuadrupedalGaitGenerator {
 public:
  // Destructor
  ~QuadrupedalGaitGenerator();

  // Constructor
  QuadrupedalGaitGenerator(double dt = 0.01, int S = 4, std::string lf = "LF_FOOT", std::string lh = "LH_FOOT",
                           std::string rf = "RF_FOOT", std::string rh = "RH_FOOT");

  std::shared_ptr<ContactSchedule> walk(StdVec_Map_string_SE3 contacts, int N_ds, int N_ss, int N_uds, int N_uss,
                                        double stepHeight, bool startPhase, bool endPhase);

  std::shared_ptr<ContactSchedule> trot(StdVec_Map_string_SE3 contacts, int N_ds, int N_ss, int N_uds, int N_uss,
                                        double stepHeight, bool startPhase, bool endPhase);

  double dt_;                              // Node dt.
  int S_;                                  // Number of contacts
  std::string lf_;                         // Left front name
  std::string lh_;                         // Left hind name
  std::string rf_;                         // Right front name
  std::string rh_;                         // Right hind name
  std::vector<std::string> contactNames_;  // Contact names list.

 private:
};

#endif  // GAIT_HPP
