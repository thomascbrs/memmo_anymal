#include "GaitManager.hpp"
// #include <pinocchio/algorithm/frames.hpp>
// #include <cstdlib>
#include <stdlib.h>

GaitManager::GaitManager(const pinocchio::Model &model, const VectorN &q) : initial_schedule_(), default_schedule_() {
  params_ = Params();
  initialize(model, q);
}

GaitManager::GaitManager(const pinocchio::Model &model, const VectorN &q, const Params &params)
    : params_(params), initial_schedule_(), default_schedule_() {
  initialize(model, q);
}

void GaitManager::initialize(const pinocchio::Model &model, const VectorN &q) {
  // contactNames_ = {lf, lh, rf, rh};
  // contactNames_sl1m_ = {lf, rf, lh, rh};

  contactNames_ = params_.feet_names;
  contactNames_sl1m_ = params_.feet_names_sl1m;

  for (auto elem : contactNames_) {
    std ::cout << "params_.feet_names" << elem << std::endl;
  }
  // Contcat names always in this order.
  // order matters! LF LH RF RH
  const std::string lf = contactNames_[0];
  const std::string lh = contactNames_[1];
  const std::string rf = contactNames_[2];
  const std::string rh = contactNames_[3];

  // Update parameters with the parameters
  type_ = params_.type;
  dt_ = params_.dt;
  N_ss_ = params_.N_ss;
  N_ds_ = params_.N_ds;
  N_uds_ = params_.N_uds;
  N_uss_ = params_.N_uss;
  nsteps_ = params_.nsteps;
  stepHeight_ = params_.stepHeight;
  N_phase_return_ = params_.N_phase_return;

  pinocchio::Data data(model);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // std::map<std::string, pinocchio::SE3> cs0, cs1;

  // Populate cs0 with SE3 transforms for each foot
  cs0[lh] = data.oMf[model.getFrameId(lh)];
  cs0[lf] = data.oMf[model.getFrameId(lf)];
  cs0[rh] = data.oMf[model.getFrameId(rh)];
  cs0[rf] = data.oMf[model.getFrameId(rf)];

  // Populate cs1 with SE3 transforms for each foot
  cs1[lh] = data.oMf[model.getFrameId(lh)];
  cs1[lf] = data.oMf[model.getFrameId(lf)];
  cs1[rh] = data.oMf[model.getFrameId(rh)];
  cs1[rf] = data.oMf[model.getFrameId(rf)];

  QuadrupedalGaitGenerator gait_generator_ = QuadrupedalGaitGenerator(dt_, 4, lf, lh, rf, rh);
  if (type_ == "trot") {
    initial_schedule_ = gait_generator_.trot({cs0, cs1}, 50, N_ss_, N_uss_, N_uds_, stepHeight_, true, false);
    default_schedule_ = gait_generator_.trot({cs0, cs1}, N_ds_, N_ss_, N_uss_, N_uds_, stepHeight_, false, false);
  } else if (type_ == "walk") {
    initial_schedule_ = gait_generator_.walk({cs0, cs1}, 100, N_ss_, N_uss_, N_uds_, stepHeight_, true, false);
    default_schedule_ = gait_generator_.walk({cs0, cs1}, N_ds_, N_ss_, N_uss_, N_uds_, stepHeight_, false, false);
  } else {
    throw std::invalid_argument("Unknown gait type in the config file. Try Trot or Walk.");
  }

  if (params_.horizon == 0) {
    // horizon of one gait.
    horizon_ = default_schedule_->T_;
  } else {
    horizon_ = params_.horizon;
  }

  timeline_ = 0;  // Current timeline
  new_step_ = false;
  is_first_gait_ = true;
  current_gait_ = MatrixN_int::Zero(4, 4);  // Initialization
  NGAIT = 20;                               // Number of switches to return, should be large enought for sl1m
  timings.push_back(0.);                    // Random number for initialisation.

  // Send new step flag only 1 over n_new_steps :
  ratio_nsteps_ = 1;
  // _ratio_nsteps = 2; // trot in simu with a lot of surfaces
  new_step_counter_ = 0;

  // Next MPC gait
  cmd_gait_ = 0;  // [0 --> Default, 1 --> Walk, 2 --> Trot]
  // Register the 2 type of shcedule
  trot_schedule_ = gait_generator_.trot({cs0, cs1}, params_.trot_N_ds, params_.trot_N_ss, params_.trot_N_uss,
                                        params_.trot_N_uds, stepHeight_, false, false);
  walk_schedule_ = gait_generator_.walk({cs0, cs1}, params_.walk_N_ds, params_.walk_N_ss, params_.walk_N_uss,
                                        params_.walk_N_uds, stepHeight_, false, false);
  // Initialize trot schedule
  trot_schedule_->updateSwitches();
  std::vector<std::shared_ptr<ContactSchedule>> queue_tmp;
  queue_tmp.push_back(trot_schedule_);
  trot_switches_.clear();
  update_switches(queue_tmp, trot_switches_, 0, false);
  // Initialize walk schedule
  walk_schedule_->updateSwitches();
  queue_tmp.clear();
  queue_tmp.push_back(walk_schedule_);
  walk_switches_.clear();
  update_switches(queue_tmp, walk_switches_, 0, false);
  // Initialize default schedule
  queue_tmp.clear();
  queue_tmp.push_back(default_schedule_);
  default_switches_.clear();
  update_switches(queue_tmp, default_switches_, 0, false);

  // Add initial CS (with longer stand phase to warm-up the MPC)
  initial_schedule_->updateSwitches();
  queue_cs_.push_back(initial_schedule_);

  // Add default CS depending on the horizon length.
  std::cout << "gait manager horizon : " << horizon_ << std::endl;
  double T_global = initial_schedule_->T_;
  while (T_global < horizon_) {
    std::shared_ptr<ContactSchedule> schedule = std::make_shared<ContactSchedule>(*default_schedule_);
    schedule->updateSwitches();
    queue_cs_.insert(queue_cs_.begin(), schedule);
    T_global += schedule->T_;
  }
  // Update the switches
  current_switches_.clear();
  update_switches(queue_cs_, current_switches_, timeline_, true);
  print_switches(current_switches_);
  print_queue();

  // Update the switches and get the corresponding gait for each case
  // update_switches(current_switches_, timeline_);
}

bool GaitManager::update() {
  bool addContact = false;
  new_step_ = false;

  for (int s = 0; s < nsteps_; s++) {
    timeline_++;

    // Reset the timeline when it has reached the current contact schedule
    if (timeline_ == queue_cs_.back()->T_) {
      // Remove the executed contact schedule and reset the timeline
      timeline_ = 0;
      queue_cs_.pop_back();
      // Queue of contact modified, --> update switches
      current_switches_.clear();
      update_switches(queue_cs_, current_switches_, timeline_, true);
      // print_switches(current_switches_);
    }

    // Add a contact schedule to the queue if necessary
    int count = 0;
    for (auto cs : queue_cs_) {
      count += cs->T_;
    }
    if (count - timeline_ < (horizon_)) {
      // This copy is expensive : 0.1ms compared to 0.003ms for the rest of the
      // code
      // TODO : Create a list
      std::shared_ptr<ContactSchedule> gait = get_next_cs();
      gait->updateSwitches();
      queue_cs_.insert(queue_cs_.begin(), gait);
      // Queue of contact modified, --> update switches
      current_switches_.clear();
      update_switches(queue_cs_, current_switches_, timeline_, true);
      // print_switches(current_switches_);
      // A new contact has been added to the queue.
      addContact = true;
    }
    // print_switches(current_switches_);
    // std::cout << timeline_ << std::endl;
    if (current_switches_.find(timeline_) != current_switches_.end()) {
      // std::cout << timeline_ << std::endl;
      // print_switches(current_switches_);
      new_step_counter_++;
      if (current_switches_[timeline_] != std::vector<int>{1, 1, 1, 1}) {
        // std::cout << "\n"<< std::endl;
        // std::cout << timeline_ << std::endl;
        // std::cout << "NEW STEP" << std::endl;
        // print_switches(current_switches_);
        new_step_ = true;
        current_gait_ = compute_gait(timeline_);
        // std::cout << current_gait_ << std::endl;
      }
      // Recover current gait configuration
    }
  }

  return addContact;
}

void GaitManager::set_next_gait(const int cmd) {
  if (cmd != cmd_gait_) {
    cmd_gait_ = cmd;
    if (cmd == 1) {
      std::cout << "Gait modification : "
                << "Walking" << std::endl;
    } else if (cmd == 2) {
      std::cout << "Gait modification : "
                << "Trotting" << std::endl;
    } else {
      std::cout << "Gait modification : "
                << "Default" << std::endl;
    }
    // Queue of contact modified, --> update switches
    current_switches_.clear();
    update_switches(queue_cs_, current_switches_, timeline_, true);
  }
}

std::shared_ptr<ContactSchedule> GaitManager::get_next_cs() {
  if (cmd_gait_ == 1) {
    return std::make_shared<ContactSchedule>(*(walk_schedule_));
  } else if (cmd_gait_ == 2) {
    return std::make_shared<ContactSchedule>(*(trot_schedule_));
  } else {
    return std::make_shared<ContactSchedule>(*(default_schedule_));
  }
}
std::map<int, std::vector<int>> GaitManager::get_next_switch() {
  if (cmd_gait_ == 1) {
    return walk_switches_;
  } else if (cmd_gait_ == 2) {
    return trot_switches_;
  } else {
    return default_switches_;
  }
}

std::vector<int> GaitManager::evaluate_config(const ContactSchedule &schedule, int timeline) {
  std::vector<int> gait_tmp(4, 0);
  for (size_t c = 0; c < schedule.contactNames_.size(); ++c) {
    std::string name = schedule.contactNames_[c];
    size_t j = static_cast<size_t>(std::find(contactNames_sl1m_.begin(), contactNames_sl1m_.end(), name) -
                                   contactNames_sl1m_.begin());
    auto phases = schedule.phases_[c];
    auto active_phase = phases[0];
    auto inactive_phase = phases[1];
    if (phases.size() == 3) {
      if (active_phase->T_ - timeline - 1 > 0) {  // case 1, inside first Active phase
        gait_tmp[j] = 1;
      } else if (active_phase->T_ + inactive_phase->T_ - timeline - 1 > 0) {  // case 2, during inactive phase
        gait_tmp[j] = 0;
      } else {  // case 3, inside last Active phase
        gait_tmp[j] = 1;
      }
    }
  }
  return gait_tmp;
}

void GaitManager::update_switches(const std::vector<std::shared_ptr<ContactSchedule>> cs_queue,
                                  std::map<int, std::vector<int>> &switches, const int timeline_in, bool use_next_cs) {
  // int index_cs = 0;
  int T_total = 0;
  for (auto itr = cs_queue.rbegin(); itr != cs_queue.rend(); ++itr) {
    const ContactSchedule &cs = **itr;
    for (size_t c = 0; c < cs.phases_.size(); c++) {
      auto &phases = cs.phases_[c];
      int phase_index = T_total;  // Assuming all T the same .. TODO
      for (size_t p = 0; p < phases.size(); p += 2) {
        int T_active = phases[p]->T_;
        int T_inactive = 0;
        if (p + 1 < phases.size()) {
          T_inactive = phases[p + 1]->T_;
          int switch_inactive = phase_index + T_active - 1;
          int switch_active = phase_index + T_active + T_inactive - 1;
          if (switches.find(switch_active) == switches.end()) {
            if ((T_active + T_inactive - 1) == cs.T_ - 1) {
              // End of contact schedule, evaluate next contact schedule.
              // TODO : A next_schedule mechanism with a wainting list populated
              // by default if no CS.
              auto nextItr = std::next(itr);
              if (nextItr != cs_queue.rend()) {
                switches[switch_active] = evaluate_config(**nextItr, 0);
              } else {
                // TODO Avoid calling get_next_cs, create a copy that is computationally expensive
                if (use_next_cs) {
                  switches[switch_active] = evaluate_config(*get_next_cs(), 0);
                } else {
                  switches[switch_active] = evaluate_config(cs, 0);
                }
              }
            } else {
              switches[switch_active] = evaluate_config(cs, T_active + T_inactive - 1);
            }
          }
          if (switch_inactive != -1 && switches.find(switch_inactive) == switches.end()) {
            // switch_inactive = -1 --> Taken into account in the previous
            // schedule.
            switches[switch_inactive] = evaluate_config(cs, T_active - 1);
          }
        }
        phase_index += T_active + T_inactive;
      }
    }
    T_total += cs.T_;
    // index_cs += 1;
  }
  return;
}

MatrixN_int GaitManager::compute_gait(int timeline) {
  // Reset timings
  timings.clear();
  // Warning : The switches has been updated previously.
  std::vector<std::vector<int>> gait;
  auto it = current_switches_.begin();
  // This function is called only when a new step is created. timeline == one of
  // the switche key.
  if (timeline < it->first) {
    std::cout << "Calling gait at the wrong timing, not when a new step is created" << std::endl;
  }
  for (auto itr = current_switches_.begin(); itr != current_switches_.end(); ++itr) {
    if (timeline <= itr->first) {
      gait.push_back(itr->second);
      if (timings.size() == 0) {
        timings.push_back(0.);
      } else {
        timings.push_back(dt_ * static_cast<double>(itr->first - (std::prev(itr))->first));
      }
    }
  }

  // Get next Cs without copying it
  std::map<int, std::vector<int>> nxt_switch = get_next_switch();
  // print_switches(nxt_switch);

  // We use here the switches as defined in ContactSchedule, a bit different.
  size_t index = 0;
  size_t mapZise = nxt_switch.size();

  while (gait.size() < static_cast<size_t>(NGAIT)) {
    size_t mapIndex = index % mapZise;
    auto it_nxt = nxt_switch.begin();
    std::advance(it_nxt, mapIndex);  // Move 1 since it has been taken into account.

    gait.push_back(it_nxt->second);
    if (mapIndex != 0) {
      timings.push_back(dt_ * static_cast<double>(it_nxt->first - (std::prev(it_nxt))->first));
    } else {
      timings.push_back(dt_ * static_cast<double>(it_nxt->first + 1));
    }
    index += 1;
  }

  const size_t nCols = gait[0].size();
  const size_t nRows = gait.size();
  // Reset size of the current gait
  current_gait_ = MatrixN_int::Zero(static_cast<Eigen::Index>(nRows), static_cast<Eigen::Index>(nCols));
  for (size_t i = 0; i < gait.size(); ++i) {
    for (size_t j = 0; j < gait[0].size(); j++) {
      current_gait_(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = gait[i][j];
    }
  }
  return current_gait_;
}

void GaitManager::print_switches(std::map<int, std::vector<int>> &switches) {
  for (const auto &elem : switches) {
    std::cout << "Key : " << elem.first << ", Value : [";
    for (const auto &value : elem.second) {
      std::cout << value << " ";
    }
    std::cout << "]" << std::endl;
  }
}

// Easier implementation for printing
void GaitManager::print_queue() {
  std::cout << "\n"
            << "Current timeline : " << timeline_ << "\n"
            << std::endl;

  ////////////////
  // Print header
  // const char* columns_str = std::getenv("COLUMNS"); // if COLUMNS is in the
  // ENV variables. int terminal_width = columns_str ? std::atoi(columns_str) :
  // 100;
  int terminal_width = 100;
  int colWidth = (terminal_width - terminal_width / 5) / 4;
  // Compute the total width of the table
  size_t totalWidth = static_cast<size_t>(colWidth + 1) * contactNames_sl1m_.size();
  // Print the top border
  // std::cout << std::string(totalWidth, '_') << "\n" << std::endl;
  // Print the headers centered in each column
  for (const auto &header : contactNames_sl1m_) {
    size_t padding = static_cast<size_t>(colWidth) - header.length();
    size_t leftPadding = padding / 2;
    size_t rightPadding = padding - leftPadding;
    std::cout << std::string(leftPadding, ' ') << header << std::string(rightPadding, ' ');
    std::cout << "|";
  }
  std::cout << std::endl;
  // Print the bottom border
  std::cout << std::string(totalWidth, '_') << std::endl;

  // Iterating over the contact schedules.
  int totalLines = 10;  // total number of line for a schedule

  for (auto itr = queue_cs_.rbegin(); itr != queue_cs_.rend(); ++itr) {
    const ContactSchedule &cs = **itr;
    for (size_t p = 0; p < cs.phases_[0].size(); p++) {
      for (const auto &foot_name : contactNames_sl1m_) {
        auto it = std::find(cs.contactNames_.begin(), cs.contactNames_.end(), foot_name);
        if (it == cs.contactNames_.end()) {
          throw std::runtime_error("Naming not consistent between GaitManager and ContactSchedule.");
        }
        size_t index = static_cast<size_t>(std::distance(cs.contactNames_.begin(), it));
        auto &phases = cs.phases_[index];
        std::string text = "";
        p % 2 == 0 ? text += "Active : " : text += "Inactive :";
        text += std::to_string(phases[p]->T_);
        size_t padding = static_cast<size_t>(colWidth) - text.length();
        size_t leftPadding = padding / 2;
        size_t rightPadding = padding - leftPadding;
        std::cout << std::string(leftPadding, ' ') << text << std::string(rightPadding, ' ');
        std::cout << "|";
      }
      std::cout << std::endl;
      if (p != cs.phases_[0].size() - 1) {
        std::cout << std::string(totalWidth, '-') << std::endl;
      }
    }
    std::cout << std::string(totalWidth, '_') << std::endl;
  }
}

std::vector<std::vector<std::vector<CoeffBezier>>> GaitManager::get_coefficients() {
  std::vector<std::vector<std::vector<CoeffBezier>>> coeffs;
  for (auto itr = queue_cs_.rbegin(); itr != queue_cs_.rend(); ++itr) {
    const ContactSchedule &cs = **itr;
    std::vector<std::vector<CoeffBezier>> cs_list;
    for (size_t c = 0; c < cs.contactNames_.size(); ++c) {
      std::vector<CoeffBezier> foot_list;
      auto &phases = cs.phases_[c];
      for (size_t p = 0; p < phases.size(); p += 2) {
        if (p + 1 < phases.size()) {
          foot_list.push_back(
              CoeffBezier(phases[p + 1]->trajectory_->getT0(), phases[p + 1]->trajectory_->get_coefficients()));
        }
      }
      cs_list.push_back(foot_list);
    }
    coeffs.push_back(cs_list);
  }
  return coeffs;
}

// Try to vary the size of the boxes depending on the size of the phase.
// void GaitManager::print_queue(){
//     std::cout << "\n" << "Current timeline : " << timeline_ << "\n" <<
//     std::endl;

//     ////////////////
//     // Print header
//     const char* columns_str = std::getenv("COLUMNS"); // if COLUMNS is in the
//     ENV variables. int terminal_width = columns_str ? std::atoi(columns_str)
//     : 100; int colWidth = (terminal_width - terminal_width/5) / 4 ;
//     // Compute the total width of the table
//     size_t totalWidth = static_cast<size_t>(colWidth + 1)  *
//     contactNames_sl1m_.size();
//     // Print the top border
//     // std::cout << std::string(totalWidth, '_') << "\n" << std::endl;
//     // Print the headers centered in each column
//     for (const auto& header : contactNames_sl1m_) {
//         size_t padding = static_cast<size_t>(colWidth) - header.length();
//         size_t leftPadding = padding / 2;
//         size_t rightPadding = padding - leftPadding;
//         std::cout << std::string(leftPadding, ' ') << header <<
//         std::string(rightPadding, ' '); std::cout << "|";
//     }
//     std::cout << std::endl;
//     // Print the bottom border
//     std::cout << std::string(totalWidth, '_') << std::endl;

//     // Iterating over the contact schedules.
//     int totalLines = 10;  // total number of line for a schedule

//     for (auto itr = queue_cs_.rbegin(); itr != queue_cs_.rend(); ++itr)
//     {
//         const ContactSchedule &cs = **itr;
//         const double factor = (double)totalLines / (double)cs.T_;
//         // For each line in total line per schedule, check what should be
//         printed for (int iline = 0; iline < totalLines; iline++)
//         {
//             for (const auto &foot_name : contactNames_sl1m_)
//             {
//                 auto it = std::find(cs.contactNames_.begin(),
//                 cs.contactNames_.end(), foot_name); if (it ==
//                 cs.contactNames_.end())
//                 {
//                     throw std::runtime_error("Naming not consistent between
//                     GaitManager and ContactSchedule.");
//                 }
//                 size_t index =
//                 static_cast<size_t>(std::distance(cs.contactNames_.begin(),
//                 it)); auto &phases = cs.phases_[index]; int s_cum = 0; for
//                 (size_t p = 0; p < phases.size(); p += 2)
//                 {
//                     int s_act = int(factor * (double)phases[p]->T_);
//                     int s_inact = 0.;
//                     if (p + 1 < phases.size())
//                     {
//                         s_inact = int(factor * (double)phases[p + 1]->T_);
//                     }
//                     if (iline > s_cum + s_act + s_inact){
//                         break;
//                     }
//                     // std::cout << s_act << std::endl;
//                     // std::cout << s_inact << std::endl;
//                     if (s_act > 0){
//                         if (iline == s_cum + s_act/2){
//                             std::string text = "Active : " +
//                             std::to_string(phases[p]->T_); size_t padding =
//                             static_cast<size_t>(colWidth) - text.length();
//                             size_t leftPadding = padding / 2;
//                             size_t rightPadding = padding - leftPadding;
//                             std::cout << std::string(leftPadding, ' ') <<
//                             text << std::string(rightPadding, ' '); std::cout
//                             << "|";
//                         }
//                         else if (iline == s_cum + s_act){
//                             std::cout << std::string(colWidth, '-');
//                         }
//                         else {
//                             std::cout << std::string(colWidth, ' ');
//                         }
//                     }
//                     if (s_inact > 0 and iline > s_cum + s_act){
//                         if (iline == s_cum + s_act + s_inact/2){
//                             std::string text = "Inactive : " +
//                             std::to_string(phases[p+1]->T_); size_t padding =
//                             static_cast<size_t>(colWidth) - text.length();
//                             size_t leftPadding = padding / 2;
//                             size_t rightPadding = padding - leftPadding;
//                             std::cout << std::string(leftPadding, ' ') <<
//                             text << std::string(rightPadding, ' '); std::cout
//                             << "|";
//                         }
//                         else if (iline == s_cum + s_inact + s_act){
//                             std::cout << std::string(colWidth, '-');
//                         }
//                         else {
//                             std::cout << std::string(colWidth, ' ');
//                         }
//                     }
//                     s_cum += s_act + s_inact;

//                 }
//             }
//             std::cout << std::endl;
//         }
//         std::cout << std::string(totalWidth, '_') << std::endl;
//     }
// }

// Eigen::Matrix<double, 2, 4> GaitManager::swicthes_to_gait(const std::map<int,
// std::vector<int>> &switches)
// {
// Current gait :
// std::vector<std::vector<int>> gait;
// int timeline = 0;
// const ContactSchedule& cs;
// if (timeline_in < _queue_cs.back()->T - 1){
//     int timeline = timeline_in;
//     cs = *queue_cs_.back();
// }
// else{
//     timeline = 0;
//     cs = *queue_cs_.end()[-2];
// }

// std::vector<int> init_gait = _evaluate_config(cs, timeline);
// if (init_gait != std::vector<int>{1, 1, 1, 1}) {
//     gait.push_back(init_gait);
// }
//     std::vector<int> s_list;
//     for (auto itr = switches.begin(); itr != switches.end(); ++itr)
//     {
//         s_list.push_back(itr->first);
//     }
//     std::sort(s_list.begin(), s_list.end());

//     for (size_t k = 0; k < s_list.size(); k++)
//     {
//         // Remove 4 feet on the ground, useless for SL1M.
//         if (switches[s_list[k]] != std::vector<float>{1, 1, 1, 1})
//         {
//             gait.push_back(switches[s_list[k]]);
//         }
//     }

//     return gait;
// }
