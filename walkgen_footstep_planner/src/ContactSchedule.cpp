#include "ContactSchedule.hpp"
#include <stdexcept>

ContactSchedule::ContactSchedule(double dt, int T, int S_total, std::vector<std::string> contactNames)
    : dt_(dt), T_(T), 
    S_total_(S_total), 
    C_(0),
    contactNames_(std::move(contactNames)), 
    phases_(contactNames_.size()),
    switches_() {
    std::sort(contactNames_.begin(), contactNames_.end());
    C_ = int(contactNames_.size());
}

void ContactSchedule::updateSwitches() {
    switches_.clear();
    for (int c = 0; c < C_; ++c) {
        std::cout  << "c : " << c << std::endl;
        const auto& phases = phases_[c];
        int phase_index = 0;
        for (int p = 0; p < phases.size(); p += 2) {
            const auto& phase_active = phases[p];
            int T_active = phase_active->T_;
            int T_inactive = 0;
            if (p + 1 < phases.size()) {
                const auto& phase_inactive = phases[p + 1];
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

// void ContactSchedule::addSchedule(const std::string& name, const std::vector<std::shared_ptr<ContactPhase>>& schedule) {
//     // Check if schedule is valid
//     if (schedule.size() % 2 != 0) {
//         throw std::invalid_argument("Schedule must contain an even number of ContactPhase objects");
//     }
//     for (size_t i = 0; i < schedule.size(); ++i) {
//         if (!schedule[i]) {
//             throw std::invalid_argument("Schedule contains null ContactPhase object");
//         }
//     }

//     // Check if contact name is valid
//     auto it = std::find(contactNames_.begin(), contactNames_.end(), name);
//     if (it == contactNames_.end()) {
//         throw std::invalid_argument("Invalid contact name: " + name);
//     }
//     int c = std::distance(contactNames_.begin(), it);

//     // Check if the total number of phases is within limits
//     int N_switch = 0;
//     for (size_t p = 0; p < schedule.size(); p += 2) {
//         const auto& phase_inactive = schedule[p + 1];
//         if (phase_inactive->T_ > 0) {
//             ++N_switch;
//         }
//     }
//     if (N_switch > S_total_) {
//         throw std::invalid_argument("Total number of phases is higher than allowed");
//     }

//     // Check if the total duration is correct
//     int T = 0;
//     for (const auto& phase : schedule) {
//         T += phase->T_;
//     }
//     if (T != T_) {
//         throw std::invalid_argument("Total duration of schedule does not match ContactSchedule duration");
//     }

//     // Add the schedule
//     phases_[c] = &schedule;
//     updateSwitches();
// }

// ContactSchedule ContactSchedule::operator+(const ContactSchedule& contact_schedule) const {
//     if (contact_schedule.phases_.size() != phases_.size()) {
//         throw std::invalid_argument("Total number of phases is not compatible");
//     }
//     return ContactSchedule new_schedule(dt_, T_ + contact_schedule.T_, S_total_ + contact_schedule.S_total)
// }
