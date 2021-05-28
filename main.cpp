#include <iostream>
#include "run_planner.hpp"
#include "global_path.hpp"

using namespace acsr;


BOOST_GEOMETRY_REGISTER_EIGEN_MATRIX_CS(cs::cartesian)
BOOST_GEOMETRY_REGISTER_NODE_CS(cs::cartesian)

int main() {

    //dynamicTest();
    Logger::instance().setLogLevel(LVL_INFO);
    RunPlanner<3> run_planner;
    run_planner.init();
    run_planner.run();


    //run_planner.run();
    //std::cout << "Hello, World!" << std::endl;
    /*
    int n_wire = 3;
    Eigen::VectorXd init_state(2*n_wire),target_state(2*n_wire);
    init_state<<120,120,300,1500,1500,900;
    target_state<<900,1500,900,900,300,150;
    init_state*=1e-6;
    target_state*=1e-6;
    GlobalPath global_path;
    auto nanowire_config = std::make_shared<NanowireConfig>();
    nanowire_config->readFile("config/nanowire.cfg");
    global_path.init(init_state,target_state,nanowire_config);
    global_path.getStartTargetElectordes();
    VariablesGrid states;
    global_path.generateBestReferenceTrajectory(states);*/



    return 0;
}
