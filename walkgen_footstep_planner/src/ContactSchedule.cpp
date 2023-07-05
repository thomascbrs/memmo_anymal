#include "ContactSchedule.hpp"
#include <stdexcept>

ContactSchedule::ContactSchedule(double dt, int T, int S_total, std::vector<std::string> contactNames)
    : dt_(dt),
      T_(T),
      S_total_(S_total),
      C_(0),
      contactNames_(std::move(contactNames)),
      phases_(contactNames_.size()),
      switches_() {
  std::sort(contactNames_.begin(), contactNames_.end());
  C_ = int(contactNames_.size());
}

ContactSchedule::ContactSchedule(const ContactSchedule &other)
    : dt_(other.dt_),
      T_(other.T_),
      S_total_(other.S_total_),
      C_(other.C_),
      contactNames_(other.contactNames_),
      switches_(other.switches_) {
  // Copy the phases for each contact
  if (!other.phases_.empty()) {
    for (size_t i = 0; i < other.phases_.size(); i++) {
      std::vector<std::shared_ptr<ContactPhase>> phases_i;
      for (size_t p = 0; p < other.phases_[i].size(); p += 2) {
        // std::shared_ptr<ContactPhase> phase_cp =
        // std::make_shared<ContactPhase>(*other.phases_[i][p]);
        // phases_i.push_back(phase_cp);
        phases_i.push_back(std::make_shared<ContactPhase>(*other.phases_[i][p]));
        if (p + 1 < other.phases_[i].size()) {
          phases_i.push_back(std::make_shared<ContactPhase>(*other.phases_[i][p + 1]));
        }
      }
      phases_.push_back(phases_i);
    }
  }
}

void ContactSchedule::updateSwitches() {
  switches_.clear();
  for (int c = 0; c < C_; ++c) {
    const auto &phases = phases_[static_cast<size_t>(c)];
    int phase_index = 0;
    for (size_t p = 0; p < phases.size(); p += 2) {
      const auto &phase_active = phases[p];
      int T_active = phase_active->T_;
      int T_inactive = 0;
      if (p + 1 < phases.size()) {
        const auto &phase_inactive = phases[p + 1];
        T_inactive = phase_inactive->T_;
        int switch_time = phase_index + T_active + T_inactive - 1;
        auto it = switches_.find(switch_time);
        if (it != switches_.end()) {
          it->second.push_back(c);
        } else {
          switches_[switch_time] = {c};
        }
      }
      phase_index += T_active + T_inactive;
    }
  }
}

void ContactSchedule::addSchedule(const std::string &name,
                                  const std::vector<std::shared_ptr<ContactPhase>> &schedule) {
  // Check if schedule is valid
  // if (schedule.size() % 2 != 0) {
  //     throw std::invalid_argument("Schedule must contain an even number of
  //     ContactPhase objects");
  // }
  for (size_t i = 0; i < schedule.size(); ++i) {
    if (!schedule[i]) {
      throw std::invalid_argument("Schedule contains null ContactPhase object");
    }
  }

  // Check if contact name is valid
  auto it = std::find(contactNames_.begin(), contactNames_.end(), name);
  if (it == contactNames_.end()) {
    throw std::invalid_argument("Invalid contact name: " + name);
  }
  size_t c = static_cast<size_t>(std::distance(contactNames_.begin(), it));

  // Check if the total number of phases is within limits
  int N_switch = 0;
  for (size_t p = 0; p < schedule.size(); p += 2) {
    if (!schedule[p]) {
      throw std::invalid_argument("Schedule contains null ContactPhase object");
    }
    if (p + 1 < schedule.size()) {
      const auto &phase_inactive = schedule[p + 1];
      if (phase_inactive->T_ > 0) {
        N_switch += 1;
      }
    }
  }
  if (N_switch > S_total_) {
    throw std::invalid_argument("Total number of phases is higher than allowed");
  }

  // Check if the total duration is correct
  int T = 0;
  for (const auto &phase : schedule) {
    T += phase->T_;
  }
  if (T != T_) {
    throw std::invalid_argument("Total duration of schedule does not match ContactSchedule duration");
  }

  // Add the schedule
  phases_[c] = schedule;
  updateSwitches();
}

void ContactSchedule::checkSchedule() {
  for (size_t p = 0; phases_.size(); p = +1) {
    if (phases_[p].size() == 0) {
      throw std::runtime_error("The contact-phase for " + contactNames_[p] + " hasn't been defined");
    }
  }
}

std::shared_ptr<ContactSchedule> ContactSchedule::operator+(const ContactSchedule &contact_schedule) {
  if (contact_schedule.phases_.size() != phases_.size()) {
    throw std::invalid_argument("Total number of phases is not compatible");
  }
  if (contact_schedule.contactNames_ != contactNames_) {
    throw std::invalid_argument("Contact names not compatible");
  }
  if (contact_schedule.dt_ != dt_) {
    throw std::invalid_argument("dt not compatible");
  }
  // Update both schedule switches
  updateSwitches();
  auto result = std::make_shared<ContactSchedule>(*this);

  result->T_ += contact_schedule.T_;
  result->S_total_ += contact_schedule.S_total_;

  for (size_t c = 0; c < contactNames_.size(); ++c) {
    const auto &phases_in = contact_schedule.phases_[c];
    bool is_terminal_phase_inactive = phases_[c].size() % 2;
    for (size_t p = 0; p < phases_in.size(); ++p) {
      if (is_terminal_phase_inactive && p == 0) {
        continue;
        // phases_result.back().insert(result->phases_[c].back().end(),
        // phases_[p].begin(), phases_[p].end());
      } else {
        result->phases_[c].push_back(phases_in[p]);
      }
    }
  }
  for (const auto &switch_pair : contact_schedule.switches_) {
    const auto &k = switch_pair.first;
    const auto &d = switch_pair.second;
    result->switches_[k] = d;
  }

  return result;
}
