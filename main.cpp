#include <iostream>
#include "run_planner.hpp"
#include "global_path.hpp"
using namespace acsr;


void dynamicTest(){
    const int NANOWIRE_COUNT = 3;

    PlannerConfig::readFile("config/planner.cfg");
    SystemConfig::readFile("config/system.cfg");
    NanowireConfig::readFile("config/nanowire.cfg");

    std::vector<double> _height(NANOWIRE_COUNT,500e-6);
    std::vector<double> zeta(2*NANOWIRE_COUNT,0.6);
    Eigen::Map<Eigen::Matrix<double,NANOWIRE_COUNT,1>> height_vec(_height.data());
    Eigen::Map<Eigen::Matrix<double,2*NANOWIRE_COUNT,1>> zeta_vec(zeta.data(),2*NANOWIRE_COUNT);
    auto nanowire_system = std::make_shared<NanowireSystem<NANOWIRE_COUNT,16>>(3);
    //std::shared_ptr<NanowireSystem<NANOWIRE_COUNT,16>> nanowire_system; //nanowire system
    nanowire_system->init(zeta_vec, height_vec);
    nanowire_system->reset();

    auto start = std::chrono::high_resolution_clock::now();
    for(auto i=0;i<1000;++i) {
        Eigen::Matrix<double, 2 * NANOWIRE_COUNT, 1> result;
        double duration;
        int steps = 50;
        nanowire_system->forwardPropagateBySteps(nanowire_system->randomState(), nanowire_system->randomControl(),
                                                 steps, result, duration);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    std::cout<<std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
}


BOOST_GEOMETRY_REGISTER_EIGEN_MATRIX_CS(cs::cartesian)
int main() {

    //dynamicTest();

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
