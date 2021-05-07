//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
#define NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include "nanowire_utility.hpp"

namespace acsr {
    namespace po = boost::program_options;
    //USING_NAMESPACE_ACADO

    class NanowireConfig {
    private:

        double row_space;///electrodes row space

        double column_space;///electrodes column space

        int electrodes_row;///electrodes rows

        int electrodes_column;///electrodes columns

        int field_data_rows;///the field data parameter

        int field_data_cols;///the field data parameter

        std::string data_file_name;///field data file name

        int nanowire_count;///nanowire count

        int nanowire_index_for_image;    ///the nanowire index for the path shown in image

        std::vector<double> zeta_potential_vec;///zeta potential


    public:
        /***
     * constructor
     */
        NanowireConfig() =default;

        /***
     * destructor
     */
        virtual ~NanowireConfig() =default;

        NanowireConfig(const NanowireConfig &) = delete;

        NanowireConfig operator=(const NanowireConfig &) = delete;

        /***
     * get electrodes row space
     * @return
     */
        double getRowSpace() const {
            return row_space;
        }

        /***
     * get electrodes space
     * @return
     */
        double getColumnSpace() const {
            return column_space;
        }

        /***
     * get electrodes rows
     * @return
     */
        int getElectrodesRows() const {
            return electrodes_row;
        }

        /***
     * get electrodes columns
     * @return
     */
        int getElectrodesCols() const {
            return electrodes_column;
        }

        /***
     * get field data rows(181)
     * @return
     */
        int getFieldDataRows() const {
            return field_data_rows;
        }

        /***
     * get field data columns(181)
     * @return
     */
        int getFieldDataCols() const {
            return field_data_cols;
        }

        /***
     * get field data file name
     * @return
     */
        std::string getFieldDataFileName() const {
            return data_file_name;
        }

        /***
     * get nanowire count
     * @return
     */
        int getNanowireCount() const {
            return nanowire_count;
        }

        /***
     * get the nanowire index for the path shown in image
     * @return
     */
        int getNanowirePathIndex() const {
            return nanowire_index_for_image;
        }

        /***
     * get zeta potential
     * @return
     */
        std::vector<double> getZetaPotentialVec() const {
            return zeta_potential_vec;
        }


        Eigen::Vector2d electrodePositionToPosition(const Eigen::Vector2i& electrode_position){
            if(electrode_position(0) >= electrodes_row || electrode_position(0) <0 ||
               electrode_position(1) >= electrodes_column || electrode_position(1) < 0)return {-1.0,-1.0};
            return {600e-6*electrode_position(1),600e-6*electrode_position(0)};
        }

        std::vector<Eigen::Vector2i> getNearElectrodes(const Eigen::Vector2d& pt){
            int f0 = std::floor(pt(0)/600e-6);
            int c0 = std::ceil(pt(0)/600e-6);
            int f1 = std::floor(pt(1)/600e-6);
            int c1 = std::ceil(pt(1)/600e-6);
            std::vector<Eigen::Vector2i> v={
                    {f1,f0},{c1,f0},{f1,c0},{c1,c0}
                    //{1,1},{1,1}
            };
            std::sort(v.begin(),v.end(),[](const Eigen::Vector2i& p1,const Eigen::Vector2i& p2){
                return p1(0)==p2(0)?p1(1)>p2(1):p1(0)>p2(0);
            });
            v.erase( unique( v.begin(), v.end() ), v.end() );
            v.erase(std::remove_if(v.begin(),v.end(),[&](const Eigen::Vector2i& position){
                return  (electrodePositionToPosition(position)-pt).norm()>column_space/2*std::sqrt(2.0)+10e-6;
            }),v.end());
            return v;
        }

     /***
     * read config file, this function should be called after create an instance
     */
        void readFile(std::string file_name) {
            po::options_description opt_desc("Options");
            opt_desc.add_options()
                    ("nanowire_count", po::value<int>()->default_value(3), "nanowire count")
                    ("zeta_potential", po::value<std::string>()->default_value("1 1 1 1"), "zeta potential.")
                    ("row_space", po::value<double>()->default_value(600), "row space.")
                    ("column_space", po::value<double>()->default_value(600),"column space")
                    ("electrodes_row", po::value<int>()->default_value(4), "electordes row")
                    ("electrodes_column", po::value<int>()->default_value(4),"electrodes column")
                    ("field_data_rows", po::value<int>()->default_value(181),"")
                    ("field_data_cols", po::value<int>()->default_value(181),"")
                    ("data_file_name", po::value<std::string>(),"data file")
                    ;

            po::variables_map varmap;

            std::ifstream ifs(file_name);
            if (!ifs.is_open())
                std::cout << "Nanowire Config File Not Found\n";
            else {
                po::store(po::parse_config_file(ifs, opt_desc), varmap);
                po::notify(varmap);
            }

            if (varmap.count("nanowire_count")) {
                nanowire_count = varmap["nanowire_count"].as<int>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }


            if (varmap.count("row_space")) {
                row_space = varmap["row_space"].as<double>() * 1e-6;
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            if (varmap.count("column_space")) {
                column_space = varmap["column_space"].as<double>() * 1e-6;
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            if (varmap.count("electrodes_row")) {
                electrodes_row = varmap["electrodes_row"].as<int>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            if (varmap.count("electrodes_column")) {
                electrodes_column = varmap["electrodes_column"].as<int>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            if (varmap.count("field_data_rows")) {
                field_data_rows = varmap["field_data_rows"].as<int>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            if (varmap.count("field_data_cols")) {
                field_data_cols = varmap["field_data_cols"].as<int>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            if (varmap.count("data_file_name")) {
                data_file_name = varmap["data_file_name"].as<std::string>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            zeta_potential_vec.resize(2 * nanowire_count);
            if (varmap.count("zeta_potential")) {
                std::stringstream stream(varmap["zeta_potential"].as<std::string>());
                for (auto i = 0; i < 2 * nanowire_count; ++i)
                    stream >> zeta_potential_vec[i];
            }

            std::cout << "Read Nanowire Config File Finished.\n";
        }
    };


    class EpField {
    private:
        std::shared_ptr<NanowireConfig> nanowire_config;
        double ***value;

    public:

        /***
     * bi linear interpolation
     * @param q11 value at (x1,y1)
     * @param q12 value at (x1,y2)
     * @param q21 value at(x2,y1)
     * @param q22 value at(x2,y2)
     * @param x1 first horizontal position
     * @param x2 second horizontal position
     * @param y1 first vertical position
     * @param y2 second vertical position
     * @param x interpolating x
     * @param y interpolating y
     * @return interpolating value
     */
        static double
        bilinearInterpolation(double q11, double q12, double q21, double q22, double x1, double x2, double y1,
                              double y2, double x, double y) {
            auto x2x1 = x2 - x1;
            auto y2y1 = y2 - y1;
            auto x2x = x2 - x;
            auto y2y = y2 - y;
            auto yy1 = y - y1;
            auto xx1 = x - x1;
            auto num1 = 1.0 / (x2x1 * y2y1);
            auto num2 = q11 * x2x * y2y +
                        q21 * xx1 * y2y +
                        q12 * x2x * yy1 +
                        q22 * xx1 * yy1;
            return num1 * num2;
        }

        /***
     * linear interpolation
     * @param q1
     * @param q2
     * @param x1
     * @param x2
     * @param x
     * @return
     */
        static double linearInterpolation(double q1, double q2, double x1, double x2, double x) {
            return (q2 - q1) / (x2 - x1) * (x - x1) + q1;
        }

        /***
     * interpolating for field
     * @param page
     * @param x_axis
     * @param y_axis
     * @param column_count
     * @param row_count
     * @param x
     * @param y
     * @return
     */
        double interp2(int page, double *x_axis, double *y_axis, int column_count, int row_count, double x, double y) {
            int row;
            int col = 0;
            for (row = 0; row < row_count - 1; row++) {
                if (y_axis[row] >= y) {
                    for (col = 0; col < column_count - 1; col++) {
                        if (x_axis[col] > x) {
                            break;
                        }
                    }
                    break;
                }
            }

            if (row == 0 && col == 0) {
                return value[page][0][0];
            }

            if (row == 0) {
                return linearInterpolation(value[page][0][col - 1], value[page][0][col], x_axis[col - 1], x_axis[col],
                                           x);
            }

            if (col == 0) {
                return linearInterpolation(value[page][row - 1][0], value[page][row - 1][0], y_axis[row - 1],
                                           x_axis[row], y);
            }

            return bilinearInterpolation(value[page][row - 1][col - 1], value[page][row][col - 1],
                                         value[page][row - 1][col], value[page][row][col],
                                         x_axis[col - 1], x_axis[col], y_axis[row - 1], y_axis[row], x, y);
        }


    public:

        EpField() = delete;

        /***
     * constructor
     * @param config nanowire config pointer
     */
        EpField(std::shared_ptr<NanowireConfig> config) : nanowire_config(config) {
            //read_file();
        }

        ~EpField() {
            if (value != nullptr) {
                int pages = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();
                for (int j = 0; j < 2 * pages; j++) {
                    for (int i = 0; i < nanowire_config->getFieldDataRows(); ++i)
                        delete[] value[j][i];
                    delete[] value[j];
                }
            }
        }

        /***
     * read data file
     * @return
     */
        bool readFile() {

            std::string file_str = "data/e_field/" + nanowire_config->getFieldDataFileName();
            std::ifstream myfile(file_str);

            if (!myfile.is_open()) {
                std::cout << "Open Electrical Field Data File Failed.\n";
                return false;
            } else {
                std::cout << "read file " << file_str << " done.\n";
            }

            auto pages = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();
            std::string temp;
            value = new double **[2 * pages];
            for (int j = 0; j < 2 * pages; j++) {
                value[j] = new double *[nanowire_config->getFieldDataRows()];
                for (int i = 0; i < nanowire_config->getFieldDataRows(); ++i)
                    value[j][i] = new double[nanowire_config->getFieldDataCols()];
            }
            for (int i = 0; i < nanowire_config->getFieldDataCols(); i++) {
                for (int j = 0; j < nanowire_config->getFieldDataCols(); j++) {
                    for (int k = 0; k < 2 * pages; k++)
                        myfile >> value[k][j][i];
                }
            }
            return true;
        }

        /***
     * get electrical field at position (x,y)
     * @param x
     * @param y
     * @param mat_E
     * @param wire_count
     */
        void getField(const Eigen::VectorXd &x, const Eigen::VectorXd &y, Eigen::MatrixXd &mat_E, int wire_count) {
            assert(x.size() == wire_count);
            assert(y.size() == wire_count);
            auto pages = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();
            mat_E.resize(2 * wire_count, pages);
            double ex[2 * wire_count][pages];

            auto data_columns = nanowire_config->getFieldDataCols();
            auto data_rows = nanowire_config->getFieldDataRows();


            double x_axis[data_columns];
            double y_axis[data_rows];
            for (int i = 0; i < data_columns; i++) {
                x_axis[i] = i * (nanowire_config->getElectrodesCols() - 1) * nanowire_config->getColumnSpace() /
                            (data_columns - 1);
            }

            for (int i = 0; i < data_rows; i++) {
                y_axis[i] = i * (nanowire_config->getElectrodesRows() - 1) * nanowire_config->getRowSpace() /
                            (data_rows - 1);
            }

            for (int k = 0; k < wire_count; k++) {
                int i;
                for (i = 0; i < pages; i++) {
                    ex[2 * k][i] = interp2(i, x_axis, y_axis, data_columns, data_rows, x(k), y(k));
                    ex[2 * k + 1][i] = interp2(i + pages, x_axis, y_axis, data_columns, data_rows, x(k), y(k));
                }
            }
            for (int i = 0; i < 2 * wire_count; ++i)
                mat_E.row(i) = Eigen::VectorXd::Map(&ex[i][0], pages);
        }
    };


    class Config
    {
    private:

    public:
        Config()=default;
        virtual ~Config()=default;

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
        //static double refA;
        //static double refB;
        static double quality_decrease_factor;
        static int dominant_path_count;

        //Parameters for image output.
        //static double tree_line_width;
        //static double solution_line_width;
        static int image_width;
        static int image_height;
        //static double node_diameter;
        //static double solution_node_diameter;



        /***
         * read configuration file
         * @param file_name file name
         */

        static void readFile(std::string file_name){
            po::options_description opt_desc("Options");
            opt_desc.add_options()
                    //("help","Print available options.") ("config", po::value< std::string >( &config_file_name )->default_value("../input/default.cfg"),
                    //"The name of a file to read for options (default is ../input/default.cfg). Command-line options"
                    //" override the ones in the config file. A config file may contain lines with syntax"
                    //"\n'long_option_name = value'\nand comment lines that begin with '#'." )
                    ("integration_step",po::value<double>(&Config::integration_step),"Integration step for propagations.")
                    //("stopping_type",po::value<std::string>(&Config::stopping_type),"Condition for terminating planner (iterations or time).")
                    //("stopping_check",po::value<double>(&Config::stopping_check),"Amount of time or iterations to execute.")
                    //("stats_type",po::value<std::string>(&Config::stats_type),"Condition for printing statistics of a planner (iterations or time).")
                    //("stats_check",po::value<double>(&Config::stats_check),"Frequency of statistics gathering.")
                    //("intermediate_visualization",po::value<bool>(&Config::intermediate_visualization)->default_value(false),"Flag denoting generating images during statistics gathering.")
                    ("min_time_steps",po::value<unsigned>(&Config::min_time_steps),"Minimum number of simulation steps per local planner propagation.")
                    ("max_time_steps",po::value<unsigned>(&Config::max_time_steps),"Maximum number of simulation steps per local planner propagation.")
                    ("total_time",po::value<double>(&Config::total_time),"total running time.")
                    ("random_seed",po::value<int>(&Config::random_seed),"Random seed for the planner.")
                    ("sst_delta_near",po::value<double>(&Config::sst_delta_near),"The radius for BestNear in SST.")
                    ("sst_delta_drain",po::value<double>(&Config::sst_delta_drain),"The radius for witness nodes in SST.")
                    ("planner",po::value<std::string>(),"A string for the planner to run.")
                    //("system",po::value<std::string>(),"A string for the system to plan for.")
                    ("start_state", po::value<std::string >(), "The given start state. Input is in the format of \"0 0\"")
                    ("goal_state", po::value<std::string >(), "The given goal state. Input is in the format of \"0 0\"")
                    ("goal_radius",po::value<double>(&Config::goal_radius),"The radius for the goal region.")
                    ("bidrection",po::value<bool>(&Config::bidirection)->default_value(false),"inverse process.")
                    ("optimization",po::value<bool>(&Config::optimization)->default_value(false),"optimization process.")
                    ("optimization_distance",po::value<double>(&Config::optimization_distance)->default_value(0.0),"optimization begins if distance within this value")
                    ("image_width",po::value<int>(&Config::image_width),"Width of output images.")
                    ("image_height",po::value<int>(&Config::image_height),"Height of output images.")
                    //("rrt_delta_near",po::value<double>(&Config::rrt_delta_near),"rrt star nearest range.")
                    //("rrt_delta_explore",po::value<double>(&Config::rrt_delta_explore),"rrt star explore distance.")
                    ("blossom_m",po::value<unsigned>(&Config::blossomM),"isst blossom parameter.")
                    ("blossom_n",po::value<unsigned>(&Config::blossomN),"ref-isst blossom parameter.")
                    ("dominant_path_count",po::value<int>(&Config::dominant_path_count),"ref-isst quality index count.")
                    ("quality_decrease_factor",po::value<double>(&Config::quality_decrease_factor),"isst quality parameter.")
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
                Config::init_state = Eigen::VectorXd(state.size());
                for(unsigned i=0;i<state.size();i++)
                {
                    Config::init_state[i] = state[i];
                }
            }
            state.clear();
            if (varmap.count("goal_state"))
            {
                std::stringstream stream(varmap["goal_state"].as<std::string>());
                double n; while(stream >> n) {state.push_back(n*1e-6);}
                Config::goal_state = Eigen::VectorXd(state.size());
                for(unsigned i=0;i<state.size();i++)
                {
                    Config::goal_state[i] = state[i];
                }
            }

            if (varmap.count("planner"))
            {
                auto planner_string = varmap["planner"].as<std::string>();
                boost::to_upper(planner_string);
                if(planner_string.find("REF")!=std::string::npos){
                    Config::planner = PlannerType::e_Ref_iSST;
                }else if(planner_string.find("ISST")!=std::string::npos){
                    Config::planner = PlannerType::e_iSST;
                }else if(planner_string.find("SST")!=std::string::npos){
                    Config::planner = PlannerType::e_SST;
                }
            }
            Config::sst_delta_drain*=1e-6;
            Config::sst_delta_near*=1e-6;
            Config::goal_radius*=1e-6;
            Config::optimization_distance*=1e-6;
/*
            if (varmap.count("system"))
            {
                auto system_string = varmap["system"].as<std::string>();
                boost::to_upper(system_string);
                if(system_string.find("POINT")!=std::string::npos){
                    Config::dynamic_system = DynamicSystemType::PointType;
                }else if(system_string.find("NANOWIRE")!=std::string::npos){
                    Config::dynamic_system = DynamicSystemType::NanowireType;
                }else if(system_string.find("ELECTRODE")!=std::string::npos){
                    Config::dynamic_system = DynamicSystemType::ElectrodeType;
                }
            }*/

        }
    };

    double Config::integration_step;
    //std::string Config::stopping_type;
    // Config::stopping_check;
    //std::string Config::stats_type;
    //double Config::stats_check;
    unsigned Config::min_time_steps;
    unsigned Config::max_time_steps;
    int Config::random_seed;
    double Config::sst_delta_near;
    double Config::sst_delta_drain;
    PlannerType Config::planner = PlannerType::e_Ref_iSST;
    //DynamicSystemType Config::dynamic_system = DynamicSystemType::PointType;
    Eigen::VectorXd Config::init_state;
    Eigen::VectorXd Config::goal_state;
    double Config::goal_radius =0.1;
    bool Config::bidirection = false;
    bool Config::optimization = true;
    //bool Config::intermediate_visualization = true;
    int Config::image_width=500;
    int Config::image_height=500;
    double Config::optimization_distance=0;

    //double Config::rrt_delta_near;
    //double Config::rrt_delta_explore;

    unsigned Config::blossomM = 15;
    unsigned Config::blossomN = 5;

    //double Config::refA = 1.0005;
    //double Config::refB = 1.05;
    int Config::dominant_path_count = 2;
    double Config::quality_decrease_factor = 1.5;

    double Config::total_time = 600;


}


#endif //NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
