#ifndef CONTACTSCHEDULE_HPP
#define CONTACTSCHEDULE_HPP

#include "ContactPhase.hpp"
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

class ContactSchedule {
public:
  ContactSchedule(double dt, int T, int S_total,
                  std::vector<std::string> contactNames);
  ContactSchedule(const ContactSchedule &other);
  void addSchedule(const std::string &name,
                   const std::vector<std::shared_ptr<ContactPhase>> &schedule);
  void updateSwitches();
  void checkSchedule();
  std::shared_ptr<ContactSchedule>
  operator+(const ContactSchedule &contactSchedule);

  double dt_;                             // time step
  int T_;                                 // number of nodes
  int S_total_;                           // maximum number of contact phases
  int C_;                                 // number of contacts
  std::vector<std::string> contactNames_; // contact names
  std::vector<std::vector<std::shared_ptr<ContactPhase>>>
      phases_; // phases per each contact
  std::map<int, std::vector<int>>
      switches_; // map of time node to list of contacts
};

#endif // CONTACTSCHEDULE_HPP
