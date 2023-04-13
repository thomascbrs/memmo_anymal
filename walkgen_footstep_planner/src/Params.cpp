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
    margin = 0.08;
    t_margin = 0.3;
    z_margin = 0.04;
    N_sample = 10;
    N_sample_ineq = 8;
    degree = 7;
}

Params::Params() {
    initialize_default();
}

Params::Params(const std::string& filename) {
    initialize_default();
    parse_yaml_file(filename);
}

void Params::parse_yaml_file(const std::string& filename) {
    assert_file_exists(filename);
    YAML::Node yaml_node = YAML::LoadFile(filename);

    assert_yaml_parsing(yaml_node, filename, "walkgen_params");
    const YAML::Node& config = yaml_node["walkgen_params"];

    assert_yaml_parsing(config["params"], filename, "N_phase_return");
    N_phase_return = config["params"]["N_phase_return"].as<int>();

    // assert_yaml_parsing(config, filename, "gait");
    const YAML::Node& params_gait = config["gait"];
    assert_yaml_parsing(config["gait"], filename, "type");
    assert_yaml_parsing(config["gait"], filename, "dt");
    assert_yaml_parsing(config["gait"], filename, "horizon");
    assert_yaml_parsing(config["gait"], filename, "nsteps");
    assert_yaml_parsing(config["gait"], filename, "stepHeight");
    type = config["gait"]["type"].as<std::string>();
    dt = config["gait"]["dt"].as<double>();
    horizon = config["gait"]["horizon"].as<int>();
    nsteps = config["gait"]["nsteps"].as<int>();
    stepHeight = config["gait"]["stepHeight"].as<double>();

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
    assert_yaml_parsing(config["bezier"], filename, "margin");
    assert_yaml_parsing(config["bezier"], filename, "t_margin");
    assert_yaml_parsing(config["bezier"], filename, "z_margin");
    assert_yaml_parsing(config["bezier"], filename, "N_sample_ineq");
    assert_yaml_parsing(config["bezier"], filename, "degree");

    margin = config["bezier"]["margin"].as<double>();
    t_margin = config["bezier"]["t_margin"].as<double>();
    z_margin = config["bezier"]["z_margin"].as<double>();
    N_sample = config["bezier"]["N_sample"].as<int>();
    N_sample_ineq = config["bezier"]["N_sample_ineq"].as<int>();
    degree = config["bezier"]["degree"].as<int>();

}