#include "Params.hpp"

using namespace yaml_check;

void Params::initialize_default() {
  type = "trot";
  dt = 0.01;
  horizon = 125;
  nsteps = 1;
  stepHeight = 0.15;
  N_ds = 80;
  N_ss = 35;
  N_uds = 0;
  N_uss = 0;
  N_phase_return = 2;
  margin_up = 0.1;
  t_margin_up = 0.045;
  z_margin_up = 0.045;
  margin_down = 0.1;
  t_margin_down = 0.2;
  z_margin_down = 0.2;
  N_sample = 10;
  N_sample_ineq = 8;
  degree = 7;
  feet_names = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
  feet_names_sl1m = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
  shoulder_offsets.push_back({0.367, 0.2});
  shoulder_offsets.push_back({0.367, -0.2});
  shoulder_offsets.push_back({-0.367, 0.2});
  shoulder_offsets.push_back({-0.367, -0.2});

  walk_N_ds = 90;
  walk_N_ss = 70;
  walk_N_uds = 0;
  walk_N_uss = 0;
  trot_N_ds = 40;
  trot_N_ss = 30;
  trot_N_uds = 0;
  trot_N_uss = 0;
}

Params::~Params() {}

Params::Params() { initialize_default(); }

Params::Params(const std::string &filename) {
  initialize_default();
  parse_yaml_file(filename);
}

Params::Params(const Params &other) {
  // Copy all member variables from other to this.
  this->type = other.type;
  this->dt = other.dt;
  this->horizon = other.horizon;
  this->nsteps = other.nsteps;
  this->stepHeight = other.stepHeight;
  this->N_ds = other.N_ds;
  this->N_ss = other.N_ss;
  this->N_uds = other.N_uds;
  this->N_uss = other.N_uss;
  this->N_phase_return = other.N_phase_return;
  this->feet_names = other.feet_names;
  this->feet_names_sl1m = other.feet_names_sl1m;
  this->shoulder_offsets = other.shoulder_offsets;
  this->margin_up = other.margin_up;
  this->t_margin_up = other.t_margin_up;
  this->z_margin_up = other.z_margin_up;
  this->margin_down = other.margin_down;
  this->t_margin_down = other.t_margin_down;
  this->z_margin_down = other.z_margin_down;
  this->N_sample = other.N_sample;
  this->N_sample_ineq = other.N_sample_ineq;
  this->degree = other.degree;

  // Gait parametres for changing
  this->walk_N_ds = other.walk_N_ds;
  this->walk_N_ss = other.walk_N_ss;
  this->walk_N_uds = other.walk_N_uds;
  this->walk_N_uss = other.walk_N_uss;
  this->trot_N_ds = other.trot_N_ds;
  this->trot_N_ss = other.trot_N_ss;
  this->trot_N_uds = other.trot_N_uds;
  this->trot_N_uss = other.trot_N_uss;
}

void Params::parse_yaml_file(const std::string &filename) {
  assert_file_exists(filename);
  YAML::Node yaml_node = YAML::LoadFile(filename);

  assert_yaml_parsing(yaml_node, filename, "walkgen_params");
  const YAML::Node &config = yaml_node["walkgen_params"];

  assert_yaml_parsing(config["params"], filename, "N_phase_return");
  N_phase_return = config["params"]["N_phase_return"].as<int>();

  // assert_yaml_parsing(config, filename, "gait");
  assert_yaml_parsing(config["gait"], filename, "type");
  assert_yaml_parsing(config["gait"], filename, "dt");
  assert_yaml_parsing(config["gait"], filename, "horizon");
  assert_yaml_parsing(config["gait"], filename, "nsteps");
  assert_yaml_parsing(config["gait"], filename, "stepHeight");
  assert_yaml_parsing(config["gait"], filename, "feet_names");
  assert_yaml_parsing(config["gait"], filename, "feet_names_sl1m");
  assert_yaml_parsing(config["gait"], filename, "shoulder_offsets");
  type = config["gait"]["type"].as<std::string>();
  dt = config["gait"]["dt"].as<double>();
  horizon = config["gait"]["horizon"].as<int>();
  nsteps = config["gait"]["nsteps"].as<int>();
  stepHeight = config["gait"]["stepHeight"].as<double>();
  feet_names = config["gait"]["feet_names"].as<std::vector<std::string>>();
  feet_names_sl1m = config["gait"]["feet_names_sl1m"].as<std::vector<std::string>>();
  shoulder_offsets = config["gait"]["shoulder_offsets"].as<std::vector<std::vector<double>>>();
  if (feet_names.size() != shoulder_offsets.size()) {
    throw std::runtime_error(
        "The contact name list and the shoulder offsets should "
        "have the same size.");
  }
  if (feet_names.size() != feet_names_sl1m.size()) {
    throw std::runtime_error("Both nominal contact list and sl1m should have the same size.");
  }
  for (auto elem : shoulder_offsets) {
    if (elem.size() != 2) {
      throw std::runtime_error(
          "The size of each offset in shoulder offset params "
          "should be size 2 (x and y axis).");
    }
  }
  if (type == "trot") {
    // Check the yaml file
    assert_yaml_parsing(config["gait"]["trot"], filename, "N_ds");
    assert_yaml_parsing(config["gait"]["trot"], filename, "N_ss");
    assert_yaml_parsing(config["gait"]["trot"], filename, "N_uds");
    assert_yaml_parsing(config["gait"]["trot"], filename, "N_uss");

    N_ds = config["gait"]["trot"]["N_ds"].as<int>();
    N_ss = config["gait"]["trot"]["N_ss"].as<int>();
    N_uds = config["gait"]["trot"]["N_uds"].as<int>();
    N_uss = config["gait"]["trot"]["N_uss"].as<int>();
  } else if (type == "walk") {
    // Check the yaml file
    assert_yaml_parsing(config["gait"]["walk"], filename, "N_ds");
    assert_yaml_parsing(config["gait"]["walk"], filename, "N_ss");
    assert_yaml_parsing(config["gait"]["walk"], filename, "N_uds");
    assert_yaml_parsing(config["gait"]["walk"], filename, "N_uss");

    N_ds = config["gait"]["walk"]["N_ds"].as<int>();
    N_ss = config["gait"]["walk"]["N_ss"].as<int>();
    N_uds = config["gait"]["walk"]["N_uds"].as<int>();
    N_uss = config["gait"]["walk"]["N_uss"].as<int>();
  }

  // Check the yaml file
  assert_yaml_parsing(config["bezier"], filename, "margin_up");
  assert_yaml_parsing(config["bezier"], filename, "t_margin_up");
  assert_yaml_parsing(config["bezier"], filename, "z_margin_up");
  assert_yaml_parsing(config["bezier"], filename, "margin_down");
  assert_yaml_parsing(config["bezier"], filename, "t_margin_down");
  assert_yaml_parsing(config["bezier"], filename, "z_margin_down");
  assert_yaml_parsing(config["bezier"], filename, "N_sample_ineq");
  assert_yaml_parsing(config["bezier"], filename, "degree");

  margin_up = config["bezier"]["margin_up"].as<double>();
  t_margin_up = config["bezier"]["t_margin_up"].as<double>();
  z_margin_up = config["bezier"]["z_margin_up"].as<double>();
  margin_down = config["bezier"]["margin_down"].as<double>();
  t_margin_down = config["bezier"]["t_margin_down"].as<double>();
  z_margin_down = config["bezier"]["z_margin_down"].as<double>();
  N_sample = config["bezier"]["N_sample"].as<int>();
  N_sample_ineq = config["bezier"]["N_sample_ineq"].as<int>();
  degree = config["bezier"]["degree"].as<int>();
}
