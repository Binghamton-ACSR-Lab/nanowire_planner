//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
#define NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include "nanowire_utility.hpp"
#include <boost/filesystem.hpp>
#include <assert.h>
#include <casadi/casadi.hpp>

namespace acsr {
    namespace po = boost::program_options;

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
    template<class T>
    static T
    bilinearInterpolation(T q11, T q12, T q21, T q22, T x1, T x2, T y1,
                          T y2, T x, T y) {
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
    template<class T>
    static double linearInterpolation(T q1, T q2, T x1, T x2, T x) {
        return (q2 - q1) / (x2 - x1) * (x - x1) + q1;
    }


    class NanowireConfig {
    public:

        static std::string type; ///electrodes system type
        static double electrodes_space;///electrodes row space
        static double field_height;///electrodes column space
        static int electrodes_rows;///electrodes rows
        static int electrodes_columns;///electrodes columns
        static int field_data_rows;///the field data parameter
        static int field_data_cols;///the field data parameter
        static int field_data_layers;///the field data parameter
        static double electrode_radius;// = 50
        static bool allow_to_enter_electrode;// = 0

    public:
        /***
         * constructor
         */
        NanowireConfig() = default;

        /***
         * destructor
         */
        virtual ~NanowireConfig() = default;
        NanowireConfig(const NanowireConfig &) = delete;
        NanowireConfig operator=(const NanowireConfig &) = delete;

        /***
         * get field data file name
         * @return field data file name
         */
        static std::string getFieldDataFileName(int field_dimension) {
            if (type == "cc60") {
                if(field_dimension==2)
                    return "E_Map_4by4_100V_CC60_E.txt";
                else if(field_dimension==3)
                    return "E_Map_4by4_100V_CC60_E_3D.txt";
            } else if (type == "cc600") {
                if(field_dimension==2)
                    return "E_Map_4by4Mask1_100V_CC600reE.txt";
                else if(field_dimension==3)
                    return "E_Map_4by4_100V_CC600_E_3D_900um.txt";
            }else if (type == "cc400") {
                if(field_dimension==2)
                    return "";
                else if(field_dimension==3)
                    return "E_Map_4by4_100V_CC400_E_3D_960um.txt";
            }
            return std::string();
        }

        /***
         * get convert electrode position to nanowire position
         * @param electrode_position electrode position
         * @return nanowire position
         */
        static Eigen::Vector2d electrodePositionToPosition(const Eigen::Vector2i &electrode_position) {
            if (electrode_position(0) >= electrodes_rows || electrode_position(0) < 0 ||
                electrode_position(1) >= electrodes_columns || electrode_position(1) < 0)
                return {-1.0, -1.0};
            return {electrodes_space * electrode_position(1), electrodes_space * electrode_position(0)};
        }

        /***
         * get near electrodes for a given nanowire position
         * @param pt nanowire position
         * @return near electrodes vector
         */
        static std::vector<Eigen::Vector2i> getNearElectrodes(const Eigen::Vector2d &pt,int count) {
            //std::cout << column_space;
            int f0 = std::floor(pt(0) / electrodes_space);
            int c0 = std::ceil(pt(0) / electrodes_space);
            int f1 = std::floor(pt(1) / electrodes_space);
            int c1 = std::ceil(pt(1) / electrodes_space);
            std::vector<Eigen::Vector2i> v = {
                    {f1, f0},
                    {c1, f0},
                    {f1, c0},
                    {c1, c0}
            };
            std::sort(v.begin(), v.end(), [](const Eigen::Vector2i &p1, const Eigen::Vector2i &p2) {
                return p1(0) == p2(0) ? p1(1) > p2(1) : p1(0) > p2(0);
            });
            v.erase(unique(v.begin(), v.end()), v.end());
            v.erase(std::remove_if(v.begin(), v.end(), [&](const Eigen::Vector2i &position) {
                return (electrodePositionToPosition(position) - pt).norm() > electrodes_space * std::sqrt(2.0) / 2;
            }), v.end());
            std::sort(v.begin(),v.end(),[pt](const Eigen::Vector2i &p1, const Eigen::Vector2i &p2){
                return (electrodePositionToPosition(p1) - pt).norm() < (electrodePositionToPosition(p2) - pt).norm();
            });
            return std::vector<Eigen::Vector2i>(v.begin(),v.begin()+count);
        }

        static bool isInElectrodeRegion(const Eigen::Vector2d &pt){
            auto elec_v = getNearElectrodes(pt,1);
            auto p = electrodePositionToPosition(elec_v.front());
            return (p-pt).norm()<electrode_radius;
        }

        /***
        * read config file, this function should be called after create an instance
        */
        static void readFile(std::string file_name) {
            po::options_description opt_desc("Options");
            opt_desc.add_options()
                    ("type", po::value<std::string>(&NanowireConfig::type)->default_value("cc600"), "electrode system type")
                    //("dimension", po::value<int>(&NanowireConfig::dimension)->default_value(2), "electrode system dimension")
                    ("field_height", po::value<double>(&NanowireConfig::field_height)->default_value(720), "electrode system height")
                    ("electrodes_row", po::value<int>(&NanowireConfig::electrodes_rows)->default_value(4), "electordes row")
                    ("electrodes_column", po::value<int>(&NanowireConfig::electrodes_columns)->default_value(4), "electrodes column")
                    ("field_data_rows", po::value<int>(&NanowireConfig::field_data_rows)->default_value(181), "")
                    ("field_data_cols", po::value<int>(&NanowireConfig::field_data_cols)->default_value(181), "")
                    ("field_data_layers", po::value<int>(&NanowireConfig::field_data_layers)->default_value(181), "")
                    ("electrode_radius", po::value<double>(&NanowireConfig::electrode_radius)->default_value(50), "")
                    ("allow_to_enter_electrode", po::value<bool>(&NanowireConfig::allow_to_enter_electrode)->default_value(true), "");

            po::variables_map varmap;

            std::ifstream ifs(file_name);
            if (!ifs.is_open())
                std::cout << "Nanowire Config File Not Found\n";
            else {
                po::store(po::parse_config_file(ifs, opt_desc), varmap);
                po::notify(varmap);
                std::cout<<"read nanowire config file done\n";
            }
            if(type=="cc600"){
                electrodes_space = 600e-6;
            }else if(type == "cc60"){
                electrodes_space = 60e-6;
            }else if(type == "cc400"){
                electrodes_space = 400e-6;
            }
            field_height*=1e-6;
            electrode_radius*=1e-6;
        }
    };

    std::string NanowireConfig::type;
    double NanowireConfig::electrodes_space;///electrodes row space
    double NanowireConfig::field_height;///electrodes column space
    int NanowireConfig::electrodes_rows;///electrodes rows
    int NanowireConfig::electrodes_columns;///electrodes columns
    int NanowireConfig::field_data_rows;///the field data parameter
    int NanowireConfig::field_data_cols;///the field data parameter
    int NanowireConfig::field_data_layers;///the field data parameter
    double NanowireConfig::electrode_radius;// = 50
    bool NanowireConfig::allow_to_enter_electrode;

    class EpField {
    private:
        float ***value2d;
        float ****value3d;
        float *x_axis;
        float *y_axis;
        float *z_axis;

        int _dimension;

    private:

        /***
         * interpolating for field
         * @param page
         * @param x x-position
         * @param y y-position
         * @return value from 2d interpoation
         */
        float interp2d(int page, float x, float y) {
            int col = std::ceil((NanowireConfig::field_data_cols - 1) /
                                ((NanowireConfig::electrodes_columns - 1) * NanowireConfig::electrodes_space) * x);
            int row = std::ceil((NanowireConfig::field_data_rows - 1) /
                                ((NanowireConfig::electrodes_rows - 1) * NanowireConfig::electrodes_space) * y);

            if (row == 0 && col == 0) {
                return value2d[page][0][0];
            }
            if (row == 0) {
                return linearInterpolation(value2d[page][0][col - 1], value2d[page][0][col], x_axis[col - 1], x_axis[col],x);
            }
            if (col == 0) {
                return linearInterpolation(value2d[page][row - 1][0], value2d[page][row - 1][0], y_axis[row - 1],x_axis[row], y);
            }
            return bilinearInterpolation(value2d[page][row - 1][col - 1], value2d[page][row][col - 1],
                                         value2d[page][row - 1][col], value2d[page][row][col],
                                         x_axis[col - 1], x_axis[col], y_axis[row - 1], y_axis[row], x, y);
        }

        /***
         * 3d interpolation
         * @param page page 0-32
         * @param x x-position
         * @param y y-position
         * @param z z-position
         * @return value from interpolation
         */
        float interp3d(int page, float x, float y,float z) {
            int l = std::ceil((NanowireConfig::field_data_layers - 1) / NanowireConfig::field_height * z);
            if (l == 0) {
                return interp2dWithHeight(page,x,y,0);
            }
            auto v1 = interp2dWithHeight(page,x,y,l-1);
            auto v2 = interp2dWithHeight(page,x,y,l);

            return linearInterpolation(v1,v2,z_axis[l-1],z_axis[l],z);
        }

        /***
         * intermedia function for 3d interpo
         * @param page
         * @param x x-position
         * @param y y-position
         * @param height z-height
         * @return
         */
        float interp2dWithHeight(int page, float x, float y,int height) {
            int col = std::ceil((NanowireConfig::field_data_cols - 1) /
                                ((NanowireConfig::electrodes_columns - 1) * NanowireConfig::electrodes_space) * x);
            int row = std::ceil((NanowireConfig::field_data_rows - 1) /
                                ((NanowireConfig::electrodes_rows - 1) * NanowireConfig::electrodes_space) * y);
            if (row == 0) {
                return linearInterpolation(value3d[page][0][col - 1][height], value3d[page][0][col][height], x_axis[col - 1], x_axis[col], x);
            }
            if (col == 0) {
                return linearInterpolation(value3d[page][row - 1][0][height], value3d[page][row - 1][0][height], y_axis[row - 1], x_axis[row], y);
            }
            //std::cout<<value3d[page][row - 1][col - 1][height]<<std::endl;
            return bilinearInterpolation(value3d[page][row - 1][col - 1][height], value3d[page][row][col - 1][height], value3d[page][row - 1][col][height], value3d[page][row][col][height],
                                         x_axis[col - 1], x_axis[col], y_axis[row - 1], y_axis[row], x, y);
        }


    public:
        /***
         * constructor, must specify dimension
         * @param dimension field dimension
         */
        EpField(int dimension):_dimension(dimension) {
            auto data_columns = NanowireConfig::field_data_cols;
            auto data_rows = NanowireConfig::field_data_rows;
            auto pages = NanowireConfig::electrodes_rows * NanowireConfig::electrodes_columns;
            x_axis = new float[data_columns];
            y_axis = new float[data_rows];
            for (int i = 0; i < data_columns; i++) {
                x_axis[i] = float(i * (NanowireConfig::electrodes_columns - 1) * NanowireConfig::electrodes_space /
                            (data_columns - 1));
            }
            for (int i = 0; i < data_rows; i++) {
                y_axis[i] = float(i * (NanowireConfig::electrodes_rows - 1) * NanowireConfig::electrodes_space /
                            (data_rows - 1));
            }
            if(dimension==2){
                value2d = new float **[2 * pages];
                for (int j = 0; j < 2 * pages; j++) {
                    value2d[j] = new float *[data_rows];
                    for (int i = 0; i < data_rows; ++i)
                        value2d[j][i] = new float [data_columns];
                }
            }else if(dimension==3){
                auto data_layers = NanowireConfig::field_data_layers;
                z_axis = new float[data_layers];
                for (int i = 0; i < data_layers; i++) {
                    z_axis[i] = float(i * NanowireConfig::field_height /(data_layers - 1));
                }
                value3d=new float***[2*pages];
                for(int p=0;p<2*pages;p++){
                    value3d[p] = new float**[data_rows];
                    for (int i = 0; i < data_rows; ++i) {
                        value3d[p][i] = new float *[data_columns];
                        for (int j = 0; j < data_columns; ++j) {
                            value3d[p][i][j] = new float [data_layers];
                        }
                    }
                }
            }

        }

        /***
         * deconstructor
         */
        ~EpField() {
            int pages = NanowireConfig::electrodes_rows * NanowireConfig::electrodes_columns;
            if(_dimension==2 && value2d != nullptr){
                for (int j = 0; j < 2 * pages; j++) {
                    for (int i = 0; i < NanowireConfig::field_data_rows; ++i)
                        delete[] value2d[j][i];
                    delete[] value2d[j];
                }
            }

            if(_dimension==3 && value3d != nullptr){
                for(int p=0;p<2*pages;p++){
                    for (int i = 0; i < NanowireConfig::field_data_rows; ++i) {
                        for (int j = 0; j < NanowireConfig::field_data_cols; ++j) {
                            delete[] value3d[p][i][j];
                        }
                        delete[] value3d[p][i];
                    }
                    delete[] value3d[p];
                }
            }

            delete x_axis;
            delete y_axis;
            delete z_axis;
        }

        /***
         * read data file
         * @return
         */
        bool readFile() {
            auto pages = NanowireConfig::electrodes_rows * NanowireConfig::electrodes_columns;

            std::string file_str = "data/e_field/" + NanowireConfig::getFieldDataFileName(_dimension);
            std::string binary_file_name = file_str.substr(0,file_str.length()-4)+".bin";

            auto layers = NanowireConfig::field_data_layers;
            auto rows = NanowireConfig::field_data_rows;
            auto columns = NanowireConfig::field_data_cols;

            ///if no binary file,read txt file and generate a binary file
            if (!boost::filesystem::exists(binary_file_name)) {
                std::ifstream myfile(file_str);
                if (!myfile.is_open()) {
                    std::cout << "Open Electrical Field Data File Failed.\n";
                    return false;
                } else {
                    std::cout << "read file " << file_str << " done.\n";
                }
                if(_dimension==2) {
                    for (int i = 0; i < columns; i++) {
                        for (int j = 0; j < rows; j++) {
                            for (int k = 0; k < 2 * pages; k++)
                                myfile >> value2d[k][j][i];
                        }
                    }
                    std::ofstream fil;
                    fil.open(binary_file_name, std::ios::out | std::ios::binary);
                    for(auto i=0;i<2*pages;++i){
                        for(auto j=0;j<rows;++j)
                            fil.write(reinterpret_cast<char*>(value2d[i][j]), columns*sizeof(float));
                    }
                    fil.close();
                    return true;
                }else if(_dimension==3){
                    for(auto i=0;i<layers;i++){
                        for(auto j=0;j<columns;j++){
                            for(auto m=0;m<rows;m++)
                                ///cannot apply "read" since some values are NaN
                                for(int k=0;k<2*pages;k++) {
                                    float s=0.0;
                                    if(!(myfile>>s)){
                                        myfile.clear();
                                        myfile.ignore(3);
                                    }
                                    value3d[k][m][j][i] = s;
                                }
                        }
                    }
                    myfile.close();

                    std::ofstream fil;
                    fil.open(binary_file_name, std::ios::out | std::ios::binary);
                    for(auto i=0;i<2*pages;++i){
                        for(auto j=0;j<rows;++j)
                            for(auto m=0;m<columns;++m)
                                ///write 1-D data
                                fil.write(reinterpret_cast<char*>(value3d[i][j][m]), layers*sizeof(float));
                    }
                    fil.close();
                }
            }else{ ///if exist binary file, read this file directly
                if(_dimension==2){
                    std::fstream fil;
                    fil.open(binary_file_name, std::ios::in | std::ios::binary);
                    if(fil.is_open()) {
                        for (auto i = 0; i < 2 * pages; ++i) {
                            for (auto j = 0; j < rows; ++j)
                                fil.read(reinterpret_cast<char *>(value2d[i][j]), columns * sizeof(float));
                        }
                    }
                    fil.close();

                }else if(_dimension==3){
                    std::fstream fil;
                    fil.open(binary_file_name, std::ios::in | std::ios::binary);
                    if(fil.is_open()) {
                        for (auto i = 0; i < 2 * pages; ++i) {
                            for (auto j = 0; j < rows; ++j)
                                for (auto m = 0; m < columns; ++m) {
                                    ///read only applied to 1-D data
                                    fil.read(reinterpret_cast<char *>(value3d[i][j][m]), layers * sizeof(float));
                                }
                        }
                    }
                    fil.close();
                }
            }
            return true;
        }

        /***
         * get electrical field at position (x,y)
         * @param state position, (x1,y1,x2,y2,...,xn,yn)
         * @param height a vector specify the height of each nanowire
         * @param mat_E matrix storing field
         * @param wire_count wire count
         */
        template<int WIRE_COUNT>
        void getField(const Eigen::Matrix<double,2*WIRE_COUNT,1> &state, const Eigen::Matrix<double,WIRE_COUNT,1> &height, Eigen::MatrixXd &mat_E) {
            auto pages = NanowireConfig::electrodes_rows * NanowireConfig::electrodes_columns;
            mat_E.resize(2 * WIRE_COUNT, pages);
            double ex[2 * WIRE_COUNT][pages];
            if(_dimension==2) {
                for (int k = 0; k < WIRE_COUNT; ++k) {
                    for (int i = 0; i < pages; ++i) {
                        ex[2 * k][i] = double(interp2d(i, float(state(2*k)), float(state(2*k+1))));
                        ex[2 * k + 1][i] = double(interp2d(i + pages, float(state(2*k)), float(state(2*k+1))));
                    }
                }
            }else if(_dimension==3){
                for(int k=0;k<WIRE_COUNT;++k){
                    for(int i=0;i<pages;++i){
                        ex[2*k][i]=double(interp3d(i,float(state(2*k)),float(state(2*k+1)),float(height(k))));
                        ex[2*k+1][i]=double(interp3d(i+pages,float(state(2*k)),float(state(2*k+1)),float(height(k))));
                    }
                }
            }
            ///mapping 2d-array to matrix
            mat_E = Eigen::Matrix<double,-1,-1,Eigen::RowMajor>::Map(&ex[0][0],2*WIRE_COUNT,pages);
        }

        template<int WIRE_COUNT>
        void getField(const Eigen::Matrix<double,2*WIRE_COUNT,1> &state, const Eigen::Matrix<double,WIRE_COUNT,1> &height, casadi::DM &mat_E) {
            auto pages = NanowireConfig::electrodes_rows * NanowireConfig::electrodes_columns;
            std::vector<std::vector<double>> ex;
            for(auto i=0;i<2*WIRE_COUNT;++i)
                ex.push_back(std::vector<double>(pages));            
            if(_dimension==2) {
                for (int k = 0; k < WIRE_COUNT; ++k) {
                    for (int i = 0; i < pages; ++i) {
                        ex[2 * k][i] = double(interp2d(i, float(state(2*k)), float(state(2*k+1))));
                        ex[2 * k + 1][i] = double(interp2d(i + pages, float(state(2*k)), float(state(2*k+1))));
                    }
                }
            }else if(_dimension==3){
                for(int k=0;k<WIRE_COUNT;++k){
                    for(int i=0;i<pages;++i){
                        ex[2*k][i]=double(interp3d(i,float(state(2*k)),float(state(2*k+1)),float(height(k))));
                        ex[2*k+1][i]=double(interp3d(i+pages,float(state(2*k)),float(state(2*k+1)),float(height(k))));
                    }
                }
            }
            ///mapping 2d-array to matrix
            mat_E = casadi::DM(ex);
        }
    };
}



#endif //NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
