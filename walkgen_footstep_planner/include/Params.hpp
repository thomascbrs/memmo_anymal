#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <Types.hpp>

class Params {
 public:
  // Constructor without filename.
  Params();

  // Destructor
  ~Params();

  // Constructor with filename.
  Params(const std::string &filename);

  // Copy constructor
  Params(const Params &other);

  // Initialize with default variables.
  void initialize_default();

  // Constructor with filename.
  void parse_yaml_file(const std::string &filename);

  // Gait parameters
  std::string type;
  double dt;
  int horizon;
  int nsteps;
  double stepHeight;
  int N_ds;
  int N_ss;
  int N_uds;
  int N_uss;
  int N_phase_return;
  std::vector<std::string> feet_names;
  std::vector<std::string> feet_names_sl1m;
  std::vector<std::vector<double>> shoulder_offsets;
  // Bezier parameters
  double margin_up;
  double t_margin_up;
  double z_margin_up;
  double margin_down;
  double t_margin_down;
  double z_margin_down;
  int N_sample;
  int N_sample_ineq;
  int degree;

  // Keeping default system for now.
  // Walk parameters
  int walk_N_ds;
  int walk_N_ss;
  int walk_N_uds;
  int walk_N_uss;
  // Trot parameters
  int trot_N_ds;
  int trot_N_ss;
  int trot_N_uds;
  int trot_N_uss;
};

// From
// https://gitlab.laas.fr/gepetto/quadruped-reactive-walking/-/blob/article-ral-iros-improved/include/qrw/Params.hpp
namespace yaml_check {

#define assert_yaml_parsing(yaml_node, parent_node_name, child_node_name)                                 \
  if (!yaml_node[child_node_name]) {                                                                      \
    std::ostringstream oss;                                                                               \
    oss << "Error: Wrong parsing of the YAML file from source file: [" << __FILE__ << "], in function: [" \
        << __FUNCTION__ << "], line: [" << __LINE__ << "]. Node [" << child_node_name                     \
        << "] does not exist under the node [" << parent_node_name << "].";                               \
    throw std::runtime_error(oss.str());                                                                  \
  }

////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Check if a file exists (before we try loading it)
///
/// \param[in] filename File path to check
///
////////////////////////////////////////////////////////////////////////////////////////////////
#define assert_file_exists(filename)                                                                        \
  std::ifstream f(filename.c_str());                                                                        \
  if (!f.good()) {                                                                                          \
    std::ostringstream oss;                                                                                 \
    oss << "Error: Problem opening the file [" << filename << "], from source file: [" << __FILE__          \
        << "], in function: [" << __FUNCTION__ << "], line: [" << __LINE__ << "]. The file may not exist."; \
    throw std::runtime_error(oss.str());                                                                    \
  }

}  // namespace yaml_check

#endif  // PARAMS_HPP
