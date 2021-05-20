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
    private:

        std::string type;

        int dimension;

        double row_space;///electrodes row space

        double column_space;///electrodes column space

        double height;///electrodes column space

        int electrodes_row;///electrodes rows

        int electrodes_column;///electrodes columns

        int field_data_rows;///the field data parameter

        int field_data_cols;///the field data parameter

        int field_data_layers;///the field data parameter

        std::string data_file_name;///field data file name

        int nanowire_count;///nanowire count

        int nanowire_index_for_image;    ///the nanowire index for the path shown in image

        std::vector<double> zeta_potential_vec;///zeta potential


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
     * get electrodes row space
     * @return
     */
        double getRowSpace() const {
            return row_space;
        }

        std::string getType() const {
            return type;
        }

        double getHeight() const {
            return height;
        }

        double getDimension() const{
            return dimension;
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

        int getFieldDataLayers() const{
            return field_data_layers;
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

        void setNanowireCount(int n_wire) {
            nanowire_count = n_wire;
        }

        void setZeta(const std::vector<double> &zeta) {
            assert(zeta.size() == 2 * nanowire_count);
            zeta_potential_vec = zeta;
        }

        /***
     * get zeta potential
     * @return
     */
        std::vector<double> getZetaPotentialVec() const {
            return zeta_potential_vec;
        }


        Eigen::Vector2d electrodePositionToPosition(const Eigen::Vector2i &electrode_position) {
            if (electrode_position(0) >= electrodes_row || electrode_position(0) < 0 ||
                electrode_position(1) >= electrodes_column || electrode_position(1) < 0)
                return {-1.0, -1.0};
            return {column_space * electrode_position(1), column_space * electrode_position(0)};
        }

        std::vector<Eigen::Vector2i> getNearElectrodes(const Eigen::Vector2d &pt) {
            std::cout << column_space;
            int f0 = std::floor(pt(0) / column_space);
            int c0 = std::ceil(pt(0) / column_space);
            int f1 = std::floor(pt(1) / column_space);
            int c1 = std::ceil(pt(1) / column_space);
            std::vector<Eigen::Vector2i> v = {
                    {f1, f0},
                    {c1, f0},
                    {f1, c0},
                    {c1, c0}
                    //{1,1},{1,1}
            };
            std::sort(v.begin(), v.end(), [](const Eigen::Vector2i &p1, const Eigen::Vector2i &p2) {
                return p1(0) == p2(0) ? p1(1) > p2(1) : p1(0) > p2(0);
            });
            v.erase(unique(v.begin(), v.end()), v.end());
            v.erase(std::remove_if(v.begin(), v.end(), [&](const Eigen::Vector2i &position) {
                return (electrodePositionToPosition(position) - pt).norm() > column_space * std::sqrt(2.0) / 2;
            }), v.end());
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
                    ("type", po::value<std::string>()->default_value("cc600"), "electrode system type")
                    ("dimension", po::value<int>()->default_value(2), "electrode system dimension")
                    ("height", po::value<double>()->default_value(180), "electrode system height")
                    //("row_space", po::value<double>()->default_value(600), "row space.")
                    //("column_space", po::value<double>()->default_value(600),"column space")
                    ("electrodes_row", po::value<int>()->default_value(4), "electordes row")
                    ("electrodes_column", po::value<int>()->default_value(4), "electrodes column")
                    ("field_data_rows", po::value<int>()->default_value(181), "")
                    ("field_data_cols", po::value<int>()->default_value(181), "")
                    ("field_data_layers", po::value<int>()->default_value(181), "")
                    ("data_file_name", po::value<std::string>(), "data file");

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

            if (varmap.count("dimension")) {
                dimension = varmap["dimension"].as<int>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

            if (varmap.count("height")) {
                height = varmap["height"].as<double>()*1e-6;
            } else {
                std::cout << "Nanowire Config File Error\n";
            }


            if (varmap.count("type")) {
                type = varmap["type"].as<std::string>();
                //row_space = std::stod(type.substr(2))*1e-6;
                //column_space = std::stod(type.substr(2))*1e-6;
                if (type == "cc60") {
                    row_space = column_space = 60e-6;
                    if(dimension==2)
                        data_file_name = "E_Map_4by4_100V_CC60_E.txt";
                    else if(dimension==3)
                        data_file_name = "E_Map_4by4_100V_CC60_E_3D.txt";
                } else if (type == "cc600") {
                    row_space = column_space = 600e-6;
                    data_file_name = "E_Map_4by4Mask1_100V_CC600reE.txt";
                } else {
                    std::cout << "no such config type\n";
                }
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

            /*
            if (varmap.count("data_file_name")) {
                data_file_name = varmap["data_file_name"].as<std::string>();
            } else {
                std::cout << "Nanowire Config File Error\n";
            }*/

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
        float ***value;
        float ****value3d;
        float *x_axis;
        float *y_axis;
        float *z_axis;

    private:

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
        float interp2d(int page, float x, float y) {
            int col = std::ceil((nanowire_config->getFieldDataCols() - 1) /
                                ((nanowire_config->getElectrodesCols() - 1) * nanowire_config->getColumnSpace()) * x);
            int row = std::ceil((nanowire_config->getFieldDataRows() - 1) /
                                ((nanowire_config->getElectrodesRows() - 1) * nanowire_config->getRowSpace()) * y);

            if (row == 0 && col == 0) {
                return value[page][0][0];
            }
            if (row == 0) {
                return linearInterpolation(value[page][0][col - 1], value[page][0][col], x_axis[col - 1], x_axis[col],x);
            }
            if (col == 0) {
                return linearInterpolation(value[page][row - 1][0], value[page][row - 1][0], y_axis[row - 1],x_axis[row], y);
            }
            return bilinearInterpolation(value[page][row - 1][col - 1], value[page][row][col - 1],
                                         value[page][row - 1][col], value[page][row][col],
                                         x_axis[col - 1], x_axis[col], y_axis[row - 1], y_axis[row], x, y);
        }

        float interp3d(int page, float x, float y,float z) {
            int l = std::ceil((nanowire_config->getFieldDataLayers() - 1) / nanowire_config->getHeight() * z);
            if (l == 0) {
                return interp2dWithHeight(page,x,y,0);
            }
            auto v1 = interp2dWithHeight(page,x,y,l-1);
            auto v2 = interp2dWithHeight(page,x,y,l);

            return linearInterpolation(v1,v2,z_axis[l-1],z_axis[l],z);
        }

        float interp2dWithHeight(int page, float x, float y,int height) {
            int col = std::ceil((nanowire_config->getFieldDataCols() - 1) /
                                ((nanowire_config->getElectrodesCols() - 1) * nanowire_config->getColumnSpace()) * x);
            int row = std::ceil((nanowire_config->getFieldDataRows() - 1) /
                                ((nanowire_config->getElectrodesRows() - 1) * nanowire_config->getRowSpace()) * y);
            if (row == 0) {
                return linearInterpolation(value3d[page][0][col - 1][height], value3d[page][0][col][height], x_axis[col - 1], x_axis[col], x);
            }
            if (col == 0) {
                return linearInterpolation(value3d[page][row - 1][0][height], value3d[page][row - 1][0][height], y_axis[row - 1], x_axis[row], y);
            }

            return bilinearInterpolation(value3d[page][row - 1][col - 1][height], value3d[page][row][col - 1][height], value3d[page][row - 1][col][height], value3d[page][row][col][height],
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
            auto data_columns = nanowire_config->getFieldDataCols();
            auto data_rows = nanowire_config->getFieldDataRows();
            auto pages = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();
            x_axis = new float[data_columns];
            y_axis = new float[data_rows];
            for (int i = 0; i < data_columns; i++) {
                x_axis[i] = float(i * (nanowire_config->getElectrodesCols() - 1) * nanowire_config->getColumnSpace() /
                            (data_columns - 1));
            }
            for (int i = 0; i < data_rows; i++) {
                y_axis[i] = float(i * (nanowire_config->getElectrodesRows() - 1) * nanowire_config->getRowSpace() /
                            (data_rows - 1));
            }
            if(nanowire_config->getDimension()==2){
                value = new float **[2 * pages];
                for (int j = 0; j < 2 * pages; j++) {
                    value[j] = new float *[data_rows];
                    for (int i = 0; i < data_rows; ++i)
                        value[j][i] = new float [data_columns];
                }
            }else if(nanowire_config->getDimension()==3){
                auto data_layers = nanowire_config->getFieldDataLayers();
                z_axis = new float[data_layers];
                for (int i = 0; i < data_layers; i++) {
                    z_axis[i] = float(i * nanowire_config->getHeight() /(data_layers - 1));
                }
                value3d=new float***[2*pages];
                for(int p=0;p<2*pages;p++){
                    value3d[p] = new float**[data_rows];
                    for (int i = 0; i < data_rows; ++i) {
                        value3d[p][i] = new float *[data_columns];
                        for (int j = 0; j < data_layers; ++j) {
                            value3d[p][i][j] = new float [data_layers];
                        }
                    }
                }
            }

        }

        ~EpField() {
            int pages = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();
            if(nanowire_config->getDimension()==2 && value != nullptr){
                for (int j = 0; j < 2 * pages; j++) {
                    for (int i = 0; i < nanowire_config->getFieldDataRows(); ++i)
                        delete[] value[j][i];
                    delete[] value[j];
                }
            }

            if(nanowire_config->getDimension()==3 && value3d != nullptr){
                for(int p=0;p<2*pages;p++){
                    for (int i = 0; i < nanowire_config->getFieldDataRows(); ++i) {
                        for (int j = 0; j < nanowire_config->getFieldDataCols(); ++j) {
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
            auto pages = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();

            std::string file_str = "data/e_field/" + nanowire_config->getFieldDataFileName();
            std::string binary_file_name = file_str.substr(0,file_str.length()-4)+".bin";

            auto layers = nanowire_config->getFieldDataLayers();
            auto rows = nanowire_config->getFieldDataRows();
            auto columns = nanowire_config->getElectrodesCols();

            if (!boost::filesystem::exists(binary_file_name)) {
                std::ifstream myfile(file_str);
                if (!myfile.is_open()) {
                    std::cout << "Open Electrical Field Data File Failed.\n";
                    return false;
                } else {
                    std::cout << "read file " << file_str << " done.\n";
                }
                if(nanowire_config->getDimension()==2) {
                    for (int i = 0; i < columns; i++) {
                        for (int j = 0; j < rows; j++) {
                            for (int k = 0; k < 2 * pages; k++)
                                myfile >> value[k][j][i];
                        }
                    }
                    std::ofstream fil;
                    fil.open(binary_file_name, std::ios::out | std::ios::binary);
                    for(auto i=0;i<2*pages;++i){
                        for(auto j=0;j<rows;++j)
                            fil.write(reinterpret_cast<char*>(value[i][j]), columns*sizeof(float));
                    }
                    fil.close();
                    return true;
                }else if(nanowire_config->getDimension()==3){
                    for(auto i=0;i<layers;i++){
                        for(auto j=0;j<columns;j++){
                            for(auto m=0;m<rows;m++)
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
                                fil.write(reinterpret_cast<char*>(value3d[i][j][m]), layers*sizeof(float));
                    }
                    fil.close();
                    return true;
                }
            }else{
                if(nanowire_config->getDimension()==2){
                    std::fstream fil;
                    fil.open(binary_file_name, std::ios::in | std::ios::binary);
                    if(fil.is_open()) {
                        for (auto i = 0; i < 2 * pages; ++i) {
                            for (auto j = 0; j < rows; ++j)
                                fil.read(reinterpret_cast<char *>(value[i][j]), columns * sizeof(float));
                        }
                    }
                    fil.close();

                }else if(nanowire_config->getDimension()==3){
                    std::fstream fil;
                    fil.open(binary_file_name, std::ios::in | std::ios::binary);
                    if(fil.is_open()) {
                        for (auto i = 0; i < 2 * pages; ++i) {
                            for (auto j = 0; j < rows; ++j)
                                for (auto m = 0; m < columns; ++m) {
                                    fil.read(reinterpret_cast<char *>(value3d[i][j][m]), layers * sizeof(float));
                                }
                        }
                    }
                    fil.close();
                }
            }
        }

        /***
         * get electrical field at position (x,y)
         * @param x
         * @param y
         * @param mat_E
         * @param wire_count
         */
        void getField(const Eigen::VectorXd &state, const Eigen::VectorXd &height, Eigen::MatrixXd &mat_E, int wire_count) {
            auto pages = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();
            mat_E.resize(2 * wire_count, pages);
            double ex[2 * wire_count][pages];
            if(nanowire_config->getDimension()==2) {
                for (int k = 0; k < wire_count; ++k) {
                    for (int i = 0; i < pages; ++i) {
                        ex[2 * k][i] = double(interp2d(i, float(state(2*k)), float(state(2*k+1))));
                        ex[2 * k + 1][i] = double(interp2d(i + pages, float(state(2*k)), float(state(2*k+1))));
                    }
                }
            }else if(nanowire_config->getDimension()==3){
                for(int k=0;k<wire_count;++k){
                    for(int i=0;i<pages;++i){
                        ex[2*k][i]=double(interp3d(i,float(state(2*k)),float(state(2*k+1)),float(height(k))));
                        ex[2*k+1][i]=double(interp3d(i+pages,float(state(2*k)),float(state(2*k+1)),float(height(k))));
                    }
                }
            }
            mat_E = Eigen::Matrix<double,-1,-1,Eigen::RowMajor>::Map(&ex[0][0],2*wire_count,pages);
            //for (int i = 0; i < 2 * wire_count; ++i)
                //mat_E.row(i) = Eigen::VectorXd::Map(&ex[i][0], pages);
        }
    };
}



#endif //NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
