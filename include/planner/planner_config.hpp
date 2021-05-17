//
// Created by acsr on 5/17/21.
//

#ifndef NANOWIREPLANNER_PLANNER_CONFIG_HPP
#define NANOWIREPLANNER_PLANNER_CONFIG_HPP
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include "nanowire_utility.hpp"

namespace acsr{
    class PlannerConfig
    {
    private:

    public:
        PlannerConfig()=default;
        virtual ~PlannerConfig()=default;

        static double integration_step;
        //static std::string stopping_type;
        //static double stopping_check;
        //static std::string stats_type;
        //static double stats_check;
        static double total_time;

        static int random_seed;

        static PlannerType planner;
        //static DynamicSystemType dynamic_system;
        static Eigen::VectorXd init_state;
        static Eigen::VectorXd goal_state;
        static double goal_radius;
        static bool bidirection;
        static bool intermedia_control;
        static bool optimization;
        static double optimization_distance;
        //static bool intermediate_visualization;

        //parameters for rrt star
        //static double rrt_delta_near;
        //static double rrt_delta_explore;

        //parameters for SST
        static double sst_delta_near;
        static double sst_delta_drain;

        //parameters for iSST
        static unsigned min_time_steps;
        static unsigned max_time_steps;
        static unsigned blossomM;

        //parameters for ref-iSST
        static unsigned blossomN;
        static double quality_decrease_factor;
        static double quality_factor;
        static int dominant_path_count;
        static double search_p;

        //Parameters for image output.
        static int image_width;
        static int image_height;
        static bool show_node;


        static void readFile(std::string file_name){
            po::options_description opt_desc("Options");
            opt_desc.add_options()
                    //("help","Print available options.") ("config", po::value< std::string >( &config_file_name )->default_value("../input/default.cfg"),
                    //"The name of a file to read for options (default is ../input/default.cfg). Command-line options"
                    //" override the ones in the config file. A config file may contain lines with syntax"
                    //"\n'long_option_name = value'\nand comment lines that begin with '#'." )
                    ("integration_step",po::value<double>(&PlannerConfig::integration_step),"Integration step for propagations.")
                    //("stopping_type",po::value<std::string>(&Config::stopping_type),"Condition for terminating planner (iterations or time).")
                    //("stopping_check",po::value<double>(&Config::stopping_check),"Amount of time or iterations to execute.")
                    //("stats_type",po::value<std::string>(&Config::stats_type),"Condition for printing statistics of a planner (iterations or time).")
                    //("stats_check",po::value<double>(&Config::stats_check),"Frequency of statistics gathering.")
                    //("intermediate_visualization",po::value<bool>(&Config::intermediate_visualization)->default_value(false),"Flag denoting generating images during statistics gathering.")
                    ("min_time_steps",po::value<unsigned>(&PlannerConfig::min_time_steps),"Minimum number of simulation steps per local planner propagation.")
                    ("max_time_steps",po::value<unsigned>(&PlannerConfig::max_time_steps),"Maximum number of simulation steps per local planner propagation.")
                    ("total_time",po::value<double>(&PlannerConfig::total_time),"total running time.")
                    ("random_seed",po::value<int>(&PlannerConfig::random_seed),"Random seed for the planner.")
                    ("sst_delta_near",po::value<double>(&PlannerConfig::sst_delta_near),"The radius for BestNear in SST.")
                    ("sst_delta_drain",po::value<double>(&PlannerConfig::sst_delta_drain),"The radius for witness nodes in SST.")
                    ("planner",po::value<std::string>(),"A string for the planner to run.")
                    //("system",po::value<std::string>(),"A string for the system to plan for.")
                    ("start_state", po::value<std::string >(), "The given start state. Input is in the format of \"0 0\"")
                    ("goal_state", po::value<std::string >(), "The given goal state. Input is in the format of \"0 0\"")
                    ("goal_radius",po::value<double>(&PlannerConfig::goal_radius),"The radius for the goal region.")
                    ("intermedia_control",po::value<bool>(&PlannerConfig::intermedia_control)->default_value(false),"inverse process.")
                    ("bidrection",po::value<bool>(&PlannerConfig::bidirection)->default_value(false),"inverse process.")
                    ("optimization",po::value<bool>(&PlannerConfig::optimization)->default_value(false),"optimization process.")
                    ("optimization_distance",po::value<double>(&PlannerConfig::optimization_distance)->default_value(0.0),"optimization begins if distance within this value")
                    ("image_width",po::value<int>(&PlannerConfig::image_width),"Width of output images.")
                    ("image_height",po::value<int>(&PlannerConfig::image_height),"Height of output images.")
                    //("rrt_delta_near",po::value<double>(&Config::rrt_delta_near),"rrt star nearest range.")
                    //("rrt_delta_explore",po::value<double>(&Config::rrt_delta_explore),"rrt star explore distance.")
                    ("blossom_m",po::value<unsigned>(&PlannerConfig::blossomM),"isst blossom parameter.")
                    ("blossom_n",po::value<unsigned>(&PlannerConfig::blossomN),"ref-isst blossom parameter.")
                    ("dominant_path_count",po::value<int>(&PlannerConfig::dominant_path_count),"ref-isst quality index count.")
                    ("quality_factor",po::value<double>(&PlannerConfig::quality_factor)->default_value(50.0),"isst quality parameter.")
                    ("quality_decrease_factor",po::value<double>(&PlannerConfig::quality_decrease_factor)->default_value(1.1),"isst quality parameter.")
                    ("search_p",po::value<double>(&PlannerConfig::search_p)->default_value(0.5),"reference isst search selection param.")
                    ("show_node",po::value<bool>(&PlannerConfig::show_node),"show node on image.")
                    ;

            po::variables_map varmap;

            std::ifstream ifs( file_name.c_str());
            if( !ifs.is_open() )
                std::cout << "no such file." << std::endl;
            else
            {
                po::store( po::parse_config_file( ifs, opt_desc ), varmap );
                po::notify( varmap );
                std::cout << "read config file done.\n" << std::endl;
            }

            std::vector<double> state;
            if (varmap.count("start_state"))
            {
                std::stringstream stream(varmap["start_state"].as<std::string>());
                double n; while(stream >> n) {state.push_back(n*1e-6);}
                // state = varmap["start_state"].as<std::vector<double> >();
                PlannerConfig::init_state = Eigen::VectorXd(state.size());
                for(unsigned i=0;i<state.size();i++)
                {
                    PlannerConfig::init_state[i] = state[i];
                }
            }
            state.clear();
            if (varmap.count("goal_state"))
            {
                std::stringstream stream(varmap["goal_state"].as<std::string>());
                double n; while(stream >> n) {state.push_back(n*1e-6);}
                PlannerConfig::goal_state = Eigen::VectorXd(state.size());
                for(unsigned i=0;i<state.size();i++)
                {
                    PlannerConfig::goal_state[i] = state[i];
                }
            }

            if (varmap.count("planner"))
            {
                auto planner_string = varmap["planner"].as<std::string>();
                boost::to_upper(planner_string);
                if(planner_string.find("REF")!=std::string::npos){
                    PlannerConfig::planner = PlannerType::e_Ref_iSST;
                }else if(planner_string.find("ISST")!=std::string::npos){
                    PlannerConfig::planner = PlannerType::e_iSST;
                }else if(planner_string.find("SST")!=std::string::npos){
                    PlannerConfig::planner = PlannerType::e_SST;
                }
            }
            PlannerConfig::sst_delta_drain*=1e-6;
            PlannerConfig::sst_delta_near*=1e-6;
            PlannerConfig::goal_radius*=1e-6;
            PlannerConfig::optimization_distance*=1e-6;


        }
    };

    double PlannerConfig::integration_step;
    //std::string Config::stopping_type;
    // Config::stopping_check;
    //std::string Config::stats_type;
    //double Config::stats_check;
    unsigned PlannerConfig::min_time_steps;
    unsigned PlannerConfig::max_time_steps;
    int PlannerConfig::random_seed;
    double PlannerConfig::sst_delta_near;
    double PlannerConfig::sst_delta_drain;
    PlannerType PlannerConfig::planner = PlannerType::e_Ref_iSST;
    //DynamicSystemType Config::dynamic_system = DynamicSystemType::PointType;
    Eigen::VectorXd PlannerConfig::init_state;
    Eigen::VectorXd PlannerConfig::goal_state;
    double PlannerConfig::goal_radius =5;
    bool PlannerConfig::bidirection = false;
    bool PlannerConfig::intermedia_control = false;
    bool PlannerConfig::optimization = true;
    //bool Config::intermediate_visualization = true;
    int PlannerConfig::image_width=500;
    int PlannerConfig::image_height=500;
    double PlannerConfig::optimization_distance=50;


    unsigned PlannerConfig::blossomM = 15;
    unsigned PlannerConfig::blossomN = 5;

    //double Config::refA = 1.0005;
    //double Config::refB = 1.05;
    int PlannerConfig::dominant_path_count = 2;
    double PlannerConfig::quality_factor = 50.0;
    double PlannerConfig::quality_decrease_factor = 1.1;
    double PlannerConfig::search_p = 0.5;
    double PlannerConfig::total_time = 600;
    bool PlannerConfig::show_node = true;




}

#endif //NANOWIREPLANNER_PLANNER_CONFIG_HPP
