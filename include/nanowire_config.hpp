//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
#define NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include "nanowire_utility.hpp"
#include <assert.h>
namespace acsr {
    namespace po = boost::program_options;
    //USING_NAMESPACE_ACADO

    class NanowireConfig {
    private:

        std::string type;
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
                    //("row_space", po::value<double>()->default_value(600), "row space.")
                    //("column_space", po::value<double>()->default_value(600),"column space")
                    ("electrodes_row", po::value<int>()->default_value(4), "electordes row")
                    ("electrodes_column", po::value<int>()->default_value(4), "electrodes column")
                    ("field_data_rows", po::value<int>()->default_value(181), "")
                    ("field_data_cols", po::value<int>()->default_value(181), "")
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

            if (varmap.count("type")) {
                type = varmap["type"].as<std::string>();
                //row_space = std::stod(type.substr(2))*1e-6;
                //column_space = std::stod(type.substr(2))*1e-6;
                if (type == "cc60") {
                    row_space = column_space = 60e-6;
                    data_file_name = "E_Map_4by4_100V_CC60_E.txt";
                } else if (type == "cc600") {
                    row_space = column_space = 600e-6;
                    data_file_name = "E_Map_4by4Mask1_100V_CC600reE.txt";
                } else {
                    std::cout << "no such config type\n";
                }
            } else {
                std::cout << "Nanowire Config File Error\n";
            }

/*
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
*/
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
            /*int row;
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
            }*/

            int col = std::ceil((nanowire_config->getFieldDataCols() - 1) /
                                ((nanowire_config->getElectrodesCols() - 1) * nanowire_config->getColumnSpace()) * x);
            int row = std::ceil((nanowire_config->getFieldDataRows() - 1) /
                                ((nanowire_config->getElectrodesRows() - 1) * nanowire_config->getRowSpace()) * y);

            //assert(std::ceil(x_col)==col && std::ceil(y_row)==row);

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
}



#endif //NANOWIREPLANNER_NANOWIRE_CONFIG_HPP
