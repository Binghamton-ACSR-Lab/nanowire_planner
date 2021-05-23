//
// Created by acsr on 4/5/21.
//

#ifndef TRANSITIONMATRIXROUTE_UTILITY_HPP
#define TRANSITIONMATRIXROUTE_UTILITY_HPP
#include "node.hpp"
#include "svg.hpp"
#include <fstream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <SQLiteCpp/SQLiteCpp.h>

namespace acsr {
    namespace global {

        /**
         * Steer direction enum
         */
        enum SteerDirection {
            Up = 0,
            Down,
            Left,
            Right,
            Stay
        };

        /**
         * write a sparse matrix to file with binary format
         * @tparam SparseMatrix Eigen::SparseMatrix
         * @param filename file name
         * @param matrix matrix
         */
        template<class SparseMatrix>
        inline void writeSparseMatrixToBin(const std::string &filename, const SparseMatrix &matrix) {
            assert(matrix.isCompressed() == true);
            std::ofstream out(filename, std::ios::binary | std::ios::out | std::ios::trunc);
            if (out.is_open()) {
                typename SparseMatrix::Index rows, cols, nnzs, outS, innS;
                rows = matrix.rows();
                cols = matrix.cols();
                nnzs = matrix.nonZeros();
                outS = matrix.outerSize();
                innS = matrix.innerSize();

                out.write(reinterpret_cast<char *>(&rows), sizeof(typename SparseMatrix::Index));
                out.write(reinterpret_cast<char *>(&cols), sizeof(typename SparseMatrix::Index));
                out.write(reinterpret_cast<char *>(&nnzs), sizeof(typename SparseMatrix::Index));
                out.write(reinterpret_cast<char *>(&outS), sizeof(typename SparseMatrix::Index));
                out.write(reinterpret_cast<char *>(&innS), sizeof(typename SparseMatrix::Index));

                typename SparseMatrix::Index sizeIndexS = static_cast<typename SparseMatrix::Index>(sizeof(typename SparseMatrix::Index));
                typename SparseMatrix::Index sizeScalar = static_cast<typename SparseMatrix::Index>(sizeof(typename SparseMatrix::Scalar));
                out.write(reinterpret_cast<const char *>(matrix.valuePtr()), sizeScalar * nnzs);
                out.write(reinterpret_cast<const char *>(matrix.outerIndexPtr()), sizeIndexS * outS);
                out.write(reinterpret_cast<const char *>(matrix.innerIndexPtr()), sizeIndexS * nnzs);
                out.close();
            } else {
                std::cout << "Can not write to file: " << filename << std::endl;
            }
        }

        /**
         * read a sparse matrix from file with binary format
         * @tparam SparseMatrix Eigen::SparseMatrix
         * @param filename file name
         * @param matrix matrix
         */
        template<class SparseMatrix>
        inline void readSparsMatrixFromBin(const std::string &filename, SparseMatrix &matrix) {
            std::ifstream in(filename, std::ios::binary | std::ios::in);
            if (in.is_open()) {
                typename SparseMatrix::Index rows, cols, nnz, inSz, outSz;
                typename SparseMatrix::Index sizeScalar = static_cast<typename SparseMatrix::Index>(sizeof(typename SparseMatrix::Scalar));
                typename SparseMatrix::Index sizeIndex = static_cast<typename SparseMatrix::Index>(sizeof(typename SparseMatrix::Index));
                typename SparseMatrix::Index sizeIndexS = static_cast<typename SparseMatrix::Index>(sizeof(typename SparseMatrix::Index));
                in.read(reinterpret_cast<char *>(&rows ), sizeIndex);
                in.read(reinterpret_cast<char *>(&cols ), sizeIndex);
                in.read(reinterpret_cast<char *>(&nnz  ), sizeIndex);
                in.read(reinterpret_cast<char *>(&outSz), sizeIndex);
                in.read(reinterpret_cast<char *>(&inSz ), sizeIndex);

                matrix.resize(rows, cols);
                matrix.makeCompressed();
                matrix.resizeNonZeros(nnz);

                in.read(reinterpret_cast<char *>(matrix.valuePtr()), sizeScalar * nnz);
                in.read(reinterpret_cast<char *>(matrix.outerIndexPtr()), sizeIndexS * outSz);
                in.read(reinterpret_cast<char *>(matrix.innerIndexPtr()), sizeIndexS * nnz);

                matrix.finalize();
                in.close();
            } else {
                std::cout << "Can not open binary sparse matrix file: " << filename << std::endl;
            }
        }

        /**
         * get the control from state1 to state2
         * @param n_wires wire count
         * @param state1 state1
         * @param state2 state2
         * @return control
         */
        ControlType
        getElectrodesControl(int n_wires, const NanowirePositionType &state1, const NanowirePositionType &state2) {
            ControlType control = 0;
            for (auto i = 0; i < n_wires; ++i) {
                if (abs(state1[i].first - state2[i].first) + abs(state1[i].second - state2[i].second) >= 2) {
                    return 0;
                }

                if (state1[i] == state2[i]) {
                    control |= (0b01 << 2 * (4 * state1[i].first + state1[i].second));
                } else {
                    control |= (0b10 << 2 * (4 * state1[i].first + state1[i].second));
                    control |= (0b01 << 2 * (4 * state2[i].first + state2[i].second));
                }
            }
            auto c = control;
            while (c != 0) {
                if ((c & 0b11) == 0b11)return 0;
                c = c >> 2;
            }
            return control;
        }

        /**
         * check the minimum steps from index1 to index2 not greater than step
         * @param n_wires wire count
         * @param index1 state1
         * @param index2 state2
         * @param step step
         * @return true if minimum steps from index1 to index2 not greater than step
         */
        bool withinHeuristic(int n_wires, IndexType index1, IndexType index2, int step) {
            for (auto i = 0; i < n_wires; ++i) {
                auto v1 = index1 & 0xf;
                auto v2 = index2 & 0xf;
                if (abs((v1 >> 2) - (v2 >> 2)) + abs((v1 & 0b11) - (v2 & 0b11)) > step)
                    return false;
                index1 = index1 >> 4;
                index2 = index2 >> 4;
            }
            return true;
        }

        /**
         * get heristic(minimum steps) from index1 to index2
         * @param n_wires wire count
         * @param index1 state1
         * @param index2 state2
         * @return minimum steps
         */
        int getHeuristic(int n_wires, const NanowirePositionType &state1, const NanowirePositionType &state2) {
            int value = 0;
            for (auto i = 0; i < n_wires; ++i) {
                auto temp_v = abs(state1[i].first - state2[i].first) + abs(state1[i].second - state2[i].second);
                value = std::max(value, temp_v);
            }
            return value;
        }

        /**
         * get heristic(minimum steps) from index1 to index2
         * @param n_wires wire count
         * @param index1 state1
         * @param index2 state2
         * @return minimum steps
         */
        int getHeuristic(int n_wires, IndexType index1, IndexType index2) {
            int value = 0;
            for (auto i = 0; i < n_wires; ++i) {
                auto v1 = int(index1 & 0xf);
                auto v2 = int(index2 & 0xf);
                index1 = index1 >> 4;
                index2 = index2 >> 4;
                int temp_v = abs((v1 >> 2) - (v2 >> 2)) + abs((v1 & 0b11) - (v2 & 0b11));
                value = std::max(value, temp_v);
            }
            return value;
        }

        /**
         * get quality of a state
         * @param n_wire wire count
         * @param state state
         * @return quality
         */
        int getQuality(int n_wire, const NanowirePositionType &state) {
            int d1 = 1;
            int d2 = 0;
            for (auto i = 0; i < n_wire - 1; ++i) {
                for (auto j = i + 1; j < n_wire; ++j) {
                    auto t = (abs(state[i].first - state[j].first) + abs(state[i].second - state[j].second));
                    d1 *= t;
                    d2 += t;
                }
            }
            return -d1 - d2;
        }

        /**
         * get quality of a state
         * @param n_wire wire count
         * @param state state
         * @return quality
         */
        int getQuality(int n_wire, IndexType state) {
            auto v = indexToElectrodeVector(n_wire, state);
            return getQuality(n_wire, v);
        }

        /**
         * remove a leaf of a tree, if the parent of the removed leaf becomes a leaf, then it will be removed
         * @param leaf
         */
        void trimLeaf(NodePtr leaf) {
            if (!leaf->getChildren().empty())return;
            auto parent = leaf->getParent();
            parent->removeChild(leaf);
            if (parent->getChildren().empty()) {
                trimLeaf(parent);
            }
            leaf->setParent(nullptr);
            leaf.reset();
        }

        /*
        std::vector<IndexType> controlVectorProduct(const ControlVectorType& lhs,const ControlVectorType& rhs){
            std::vector<IndexType> product;
            for(auto& p1:lhs.vec){
                for(auto& p2:rhs.vec){
                    if (p1.second == 0 || p2.second == 0)continue;
                    auto c = p1.second | p2.second;
                    auto flag = true;
                    while (c != 0) {
                        if ((c & 0b11) == 0b11){
                            flag = false;
                            break;
                        }
                        c = c >> 2;
                    }
                    if(flag)
                        product.emplace_back(p1.first*rhs._size+p2.first);
                }
            }
            return product;
        }*/

        /**
         * remove the indice in a vector whose heuristic to target exceeds within_step
         * @param n_wires wire count
         * @param control_vec a vector contains <index,control>
         * @param index current index
         * @param target target
         * @param winthin_step critical step
         * @return trimmed control vector,containing indice
         */
        std::vector<std::pair<IndexType, ControlType>>
        trimControleVec(int n_wires, const ControlVectorType &control_vec, IndexType index, IndexType target,
                        int winthin_step) {
            auto h = getHeuristic(n_wires, index, target);
            if (h < winthin_step)return control_vec.vec;
            std::vector<std::pair<IndexType, ControlType>> v;
            std::copy_if(control_vec.vec.begin(), control_vec.vec.end(), std::back_inserter(v),
                         [n_wires, target, winthin_step](const std::pair<IndexType, ControlType> &element) {
                             return withinHeuristic(n_wires, element.first, target, winthin_step);
                         });
            return v;
        }

        /**
         * check two control combinable
         * @param c1 control 1
         * @param c2 control 2
         * @param total_control combined control
         * @return true if combinable
         */
        bool controlCombinable(ControlType c1, ControlType c2, ControlType &total_control) {
            auto c = c1 | c2;
            total_control = c;
            while (c != 0) {
                if ((c & 0b11) == 0b11) {
                    return false;
                }
                c = c >> 2;
            }
            return true;
        }

        /**
         * combine control vec to form a transition vec from an state
         * @param control_vecs original control vecs
         * @param state_index sub indice
         * @param sub_target sub target
         * @param divided_vec divided vec of wires
         * @param winthin_step critical steps
         * @return the vector containing the indice of next step
         */
        std::vector<IndexType> controlVectorProductWithStep(const std::vector<ControlVectorType> &control_vecs,
                                                            const std::vector<IndexType> &state_index,
                                                            const std::vector<IndexType> &sub_target,
                                                            const std::vector<int> &divided_vec, int winthin_step) {
            std::vector<IndexType> product;
            auto size = divided_vec.size();
            std::vector<std::vector<std::pair<IndexType, ControlType>>> trimed_vec(size);
            std::vector<IndexType> cols(size);

            ///trim the vecs
            for (auto i = 0; i < size; ++i) {
                cols[i] = control_vecs[i]._size;
                trimed_vec[i] = trimControleVec(divided_vec[i], control_vecs[i], state_index[i], sub_target[i],
                                                winthin_step);
                if (trimed_vec[i].empty())
                    return product;
            }

            if (size == 2) {
                ControlType total_control_1;
                for (auto &p0:trimed_vec[0]) {
                    for (auto &p1:trimed_vec[1]) {
                        if (controlCombinable(p0.second, p1.second, total_control_1))
                            product.emplace_back(p0.first * cols[1] + p1.first);
                    }
                }
            } else if (size == 3) {
                ControlType total_control_1, total_control_2;
                for (auto &p0:trimed_vec[0]) {
                    for (auto &p1:trimed_vec[1]) {
                        auto combinable = controlCombinable(p0.second, p1.second, total_control_1);
                        if (!combinable)continue;
                        auto index1 = p0.first * cols[1] + p1.first;
                        for (auto &p2:trimed_vec[2]) {
                            if (controlCombinable(total_control_1, p2.second, total_control_2)) {
                                product.emplace_back(index1 * cols[2] + p2.first);
                            }
                        }
                    }
                }
            } else if (size == 4) {
                ControlType total_control_1, total_control_2, total_control_3;
                for (auto &p0:trimed_vec[0]) {
                    for (auto &p1:trimed_vec[1]) {
                        auto combinable1 = controlCombinable(p0.second, p1.second, total_control_1);
                        if (!combinable1)continue;
                        auto index1 = p0.first * cols[1] + p1.first;
                        for (auto &p2:trimed_vec[2]) {
                            auto combinable2 = controlCombinable(total_control_1, p2.second, total_control_2);
                            if (!combinable2)continue;
                            auto index2 = index1 * cols[2] + p2.first;
                            for (auto &p3:trimed_vec[3]) {
                                if (controlCombinable(total_control_2, p3.second, total_control_3)) {
                                    product.emplace_back(index2 * cols[3] + p3.first);
                                }
                            }
                        }
                    }
                }
            }
            return product;
        }

        std::vector<IndexType> controlVectorProduct(const std::vector<ControlVectorType> &control_vecs,
                                                    const std::vector<IndexType> &state_index,
                                                    const std::vector<int> &divided_vec) {
            std::vector<IndexType> product;
            auto size = divided_vec.size();
            std::vector<IndexType> cols(size);
            if (size == 2) {
                ControlType total_control_1;
                for (auto &p0:control_vecs[0].vec) {
                    for (auto &p1:control_vecs[1].vec) {
                        if (controlCombinable(p0.second, p1.second, total_control_1))
                            product.emplace_back(p0.first * cols[1] + p1.first);
                    }
                }
            } else if (size == 3) {
                ControlType total_control_1, total_control_2;
                for (auto &p0:control_vecs[0].vec) {
                    for (auto &p1 : control_vecs[1].vec) {
                        auto combinable = controlCombinable(p0.second, p1.second, total_control_1);
                        if (!combinable)continue;
                        auto index1 = p0.first * cols[1] + p1.first;
                        for (auto &p2:control_vecs[2].vec) {
                            if (controlCombinable(total_control_1, p2.second, total_control_2)) {
                                product.emplace_back(index1 * cols[2] + p2.first);
                            }
                        }
                    }
                }
            } else if (size == 4) {
                ControlType total_control_1, total_control_2, total_control_3;
                for (auto &p0:control_vecs[0].vec) {
                    for (auto &p1:control_vecs[1].vec) {
                        auto combinable1 = controlCombinable(p0.second, p1.second, total_control_1);
                        if (!combinable1)continue;
                        auto index1 = p0.first * cols[1] + p1.first;
                        for (auto &p2:control_vecs[2].vec) {
                            auto combinable2 = controlCombinable(total_control_1, p2.second, total_control_2);
                            if (!combinable2)continue;
                            auto index2 = index1 * cols[2] + p2.first;
                            for (auto &p3:control_vecs[3].vec) {
                                if (controlCombinable(total_control_2, p3.second, total_control_3)) {
                                    product.emplace_back(index2 * cols[3] + p3.first);
                                }
                            }
                        }
                    }
                }
            }
            return product;
        }


        /**
         * export solution path to svg
         * @param n_wire wire count
         * @param path solution path
         * @param file_name svg file name
         */
        void exportSVG(int n_wire, const std::vector<IndexType> &path, std::string file_name) {
            using namespace boost::filesystem;



            const int total_width = 1800;
            acsr::Dimensions dimensions(1.1 * total_width, 1.1 * total_width);
            acsr::Layout layout(dimensions, Layout::BottomLeft);
            acsr::Document svg(layout);
            acsr::Polygon border(acsr::Stroke(1, Color::Black));
            border << acsr::Point(0, 0) << acsr::Point(dimensions.width, 0)
                   << acsr::Point(dimensions.width, dimensions.height) << acsr::Point(0, dimensions.height);
            svg << border;

            double col_space = 600;
            double row_space = 600;

            Stroke stroke(5, Color::Black);
            for (auto i = 0; i < 4; ++i) {
                Line line(acsr::Point(0.05 * total_width, 0.05 * total_width + i * row_space),
                          acsr::Point(1.05 * total_width, 0.05 * total_width + i * row_space), stroke);
                svg << line;
            }

            for (auto i = 0; i < 4; ++i) {
                Line line(acsr::Point(0.05 * total_width + i * col_space, 0.05 * total_width),
                          acsr::Point(0.05 * total_width + i * col_space, 1.05 * total_width),
                          stroke);
                svg << line;
            }

            for (auto i = 0; i < 4; ++i) {
                for (auto j = 0; j < 4; ++j) {
                    acsr::Circle circle(
                            acsr::Point(0.05 * total_width + j * col_space, 0.05 * total_width + i * row_space),
                            100, Fill(Color::Green));
                    svg << circle;
                }
            }

            auto f = [&](std::pair<int, int> pt) {
                return acsr::Point(0.05 * total_width + pt.first * col_space,
                                   0.05 * total_width + pt.second * col_space);
            };


            std::vector<acsr::Color::Defaults> color = {acsr::Color::Aqua,
                                                        acsr::Color::Black,
                                                        acsr::Color::Cyan,
                                                        acsr::Color::Green,
                                                        acsr::Color::Red,
                                                        acsr::Color::Blue,
                                                        acsr::Color::Magenta,
                                                        acsr::Color::Brown,
                                                        acsr::Color::Fuchsia,
                                                        acsr::Color::Purple};

            acsr::Stroke forward_stroke(10, acsr::Color(60, 60, 255));
            acsr::Polyline forward_path(forward_stroke);

            auto init_state = indexToElectrodeVector(n_wire, path.front());


            std::vector<acsr::Document> sub_images;
            for (auto i = 0; i < n_wire; ++i) {
                sub_images.emplace_back(svg);
            }

            for (auto i = 0; i < n_wire; ++i) {
                acsr::Stroke start_point_stroke(15, color[i]);
                acsr::Circle start_circle(f(init_state[i]), 30, acsr::Fill(), start_point_stroke);
                svg << start_circle;
                sub_images[i] << start_circle;
            }


            std::vector<NanowirePositionType> state_vec(path.size());
            std::transform(path.begin(), path.end(), state_vec.begin(), [&](IndexType index) {
                return indexToElectrodeVector(n_wire, index);
            });

            for (int i = 0; i < n_wire; ++i) {
                acsr::Stroke path_stroke(15, color[i]);
                acsr::Polyline svg_path(path_stroke);
                for (auto state:state_vec) {
                    svg_path << f(state[i]);
                }
                svg << svg_path;
                sub_images[i] << svg_path;
            }

            svg.save(file_name);
            file_name.erase(file_name.find_last_of('.'));
            create_directories(file_name);
            for (auto i = 0; i < n_wire; ++i) {
                sub_images[i].save(file_name + "/path_" + std::to_string(i) + ".svg");
            }

        }


        /**
         * write running data to data base
         * @param n_wire wire count
         * @param init_state init state
         * @param target_state target state
         * @param estimate_step heuristic from init state to target state
         * @param over_step steps that actual steps over estimate_step
         * @param divided_vec how to divide n_wire to small count
         * @param running_time computational time
         * @param path best solution path
         */
        void writeToDatabase(int n_wire,
                             const NanowirePositionType &init_state,
                             const NanowirePositionType &target_state,
                             int estimate_step,
                             int over_step,
                             const std::vector<int> &divided_vec,
                             long running_time,
                             long path_quality,
                             const std::vector<IndexType> &path) {
            SQLite::Database db("plannerDB.db", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

            //create table
            {
                SQLite::Statement query(db, "create table if not exists GlobalRoute ("
                                            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                                            "nanowire_count INTEGER NOT NULL,"
                                            "init_state TEXT NOT NULL,"
                                            "target_state TEXT NOT NULL,"
                                            "estimate_step INTEGER NOT NULL,"
                                            "over_step INTEGER,"
                                            "divided_vec TEXT NOT NULL,"
                                            "time INTEGER,"
                                            "path_quality INTEGER,"
                                            "solution_table TEXT NOT NULL"
                                            ")");
                try {
                    query.exec();
                } catch (SQLite::Exception &e) {
                    std::cout << "Create Table Error\n";
                    std::cout << e.what();
                    return;
                }
            }

            std::string solution_table_name;
            ///insert data to table
            {
                std::string query_string = "INSERT INTO GlobalRoute (nanowire_count,init_state, target_state, estimate_step,over_step,divided_vec,time,path_quality,solution_table) VALUES(?,?,?,?,?,?,?,?,?)";
                SQLite::Statement query(db, query_string);

                std::string init_string, target_string, vec_string;
                for (auto i = 0; i < n_wire - 1; ++i) {
                    init_string.append(
                            std::to_string(init_state[i].first) + " " + std::to_string(init_state[i].second) + "  ");
                    target_string.append(
                            std::to_string(target_state[i].first) + " " + std::to_string(target_state[i].second) +
                            "  ");
                }
                init_string.append(
                        std::to_string(init_state[n_wire - 1].first) + " " +
                        std::to_string(init_state[n_wire - 1].second) +
                        "  ");
                target_string.append(std::to_string(target_state[n_wire - 1].first) + " " +
                                     std::to_string(target_state[n_wire - 1].second) + "  ");

                for (auto i = 0; i < divided_vec.size() - 1; ++i) {
                    vec_string.append(std::to_string(divided_vec[i]) + " ");
                }
                vec_string.append(std::to_string(divided_vec[divided_vec.size() - 1]) + " ");

                auto start_time = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(start_time);
                std::stringstream ss;
                ss << "solution_";
                ss << std::put_time(std::localtime(&in_time_t), "%m_%d_%H_%M_%S");
                solution_table_name = ss.str();

                query.bind(1, n_wire);
                query.bind(2, init_string);
                query.bind(3, target_string);
                query.bind(4, estimate_step);
                query.bind(5, over_step);
                query.bind(6, vec_string);
                query.bind(7, running_time);
                query.bind(8, path_quality);
                query.bind(9, solution_table_name);

                try {
                    query.exec();
                } catch (SQLite::Exception &e) {
                    std::cout << "Insert Solution Error\n";
                    std::cout << e.what();
                    return;
                }
            }
/*
        {
            std::string query_string = "create table if not exists ";
            query_string += solution_table_name;
            query_string += " (id INTEGER PRIMARY KEY AUTOINCREMENT, ";
            for (auto i = 0; i < n_wire - 1; ++i) {
                query_string += "wire_" + std::to_string(i + 1) + "_x INTEGER NOT NULL,";
                query_string += "wire_" + std::to_string(i + 1) + "_y INTEGER NOT NULL,";
            }
            query_string += "wire_" + std::to_string(n_wire) + "_x INTEGER NOT NULL,";
            query_string += "wire_" + std::to_string(n_wire) + "_y INTEGER NOT NULL)";

            SQLite::Statement query(db, query_string);
            try {
                query.exec();
            } catch (SQLite::Exception &e) {
                std::cout << "Create Solution Table Error\n";
                std::cout << e.what();
                return;
            }

            std::vector<NanowirePositionType> state_vec(path.size());
            std::transform(path.begin(), path.end(), state_vec.begin(), [&](IndexType index) {
                return indexToElectrodeVector(n_wire, index);
            });
            for (auto i = 0; i < path.size(); ++i) {
                query_string = "INSERT INTO " + solution_table_name +
                               " (";//nanowire_count,init_state, target_state, estimate_step,over_step,divided_vec,time,solution_table) VALUES(?,?,?,?,?,?,?,?)";
                for (auto j = 0; j < n_wire - 1; ++j) {
                    query_string += "wire_" + std::to_string(j + 1) + "_x, wire_" + std::to_string(j + 1) + "_y, ";
                }
                query_string +=
                        "wire_" + std::to_string(n_wire) + "_x, wire_" + std::to_string(n_wire) + "_y) VALUES (";
                for (auto j = 0; j < n_wire - 1; ++j) {
                    query_string += "?, ?, ";
                }
                query_string += "?, ?)";
                SQLite::Statement query(db, query_string);
                for (auto j = 0; j < n_wire; ++j) {
                    query.bind(2 * j + 1, state_vec[i][j].first);
                    query.bind(2 * j + 2, state_vec[i][j].second);
                }
                try {
                    query.exec();
                } catch (SQLite::Exception &e) {
                    std::cout << "Insert Solution Path Error\n";
                    std::cout << e.what();
                    return;
                }
            }
        }*/
        }

        /**
         * save astar planner data to db
         * @param n_wire wire count
         * @param init_state init state
         * @param target_state target state
         * @param divided_vec nanowire divided vector
         * @param first_solution_time time of finding first solution
         * @param best_solution_time time of finding the best solution
         * @param total_running_time total running time
         * @param path best path
         */
        void writeToDatabaseAStar(int n_wire,
                                  const NanowirePositionType &init_state,
                                  const NanowirePositionType &target_state,
                                  const std::vector<int> &divided_vec,
                                  long first_solution_time,
                                  long best_solution_time,
                                  long total_running_time,
                                  int init_cost,
                                  long init_quality,
                                  int best_cost,
                                  long best_quality,
                                  const std::vector<IndexType> &path) {
            SQLite::Database db("plannerDB.db", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

            //create table
            {
                SQLite::Statement query(db, "create table if not exists AStar ("
                                            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                                            "nanowire_count INTEGER NOT NULL,"
                                            "init_state TEXT NOT NULL,"
                                            "target_state TEXT NOT NULL,"
                                            "divided_vec TEXT NOT NULL,"
                                            "first_solution_time INTEGER NOT NULL,"
                                            "best_solution_time INTEGER,"
                                            "total_running_time INTEGER,"
                                            "init_cost INTEGER,"
                                            "init_quality INTEGER,"
                                            "best_cost INTEGER,"
                                            "best_quality INTEGER,"
                                            "solution TEXT NOT NULL"
                                            ")");
                try {
                    query.exec();
                } catch (SQLite::Exception &e) {
                    std::cout << "Create Table Error\n";
                    std::cout << e.what();
                    return;
                }
            }
            {
                std::string query_string = "INSERT INTO AStar (nanowire_count,init_state, target_state,divided_vec,first_solution_time,best_solution_time,total_running_time,init_cost,init_quality,best_cost,best_quality,solution) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)";
                SQLite::Statement query(db, query_string);

                std::string init_string, target_string, vec_string;
                for (auto i = 0; i < n_wire - 1; ++i) {
                    init_string.append(
                            std::to_string(init_state[i].first) + " " + std::to_string(init_state[i].second) + "  ");
                    target_string.append(
                            std::to_string(target_state[i].first) + " " + std::to_string(target_state[i].second) +
                            "  ");
                }
                init_string.append(
                        std::to_string(init_state[n_wire - 1].first) + " " +
                        std::to_string(init_state[n_wire - 1].second) +
                        "  ");
                target_string.append(std::to_string(target_state[n_wire - 1].first) + " " +
                                     std::to_string(target_state[n_wire - 1].second) + "  ");

                for (auto i = 0; i < divided_vec.size() - 1; ++i) {
                    vec_string.append(std::to_string(divided_vec[i]) + " ");
                }
                vec_string.append(std::to_string(divided_vec[divided_vec.size() - 1]) + " ");
                std::string solution_str = std::to_string(path.front());
                for (auto i = 1; i < path.size(); ++i) {
                    solution_str += (' ' + std::to_string(path[i]));
                }
                query.bind(1, n_wire);
                query.bind(2, init_string);
                query.bind(3, target_string);
                query.bind(4, vec_string);
                query.bind(5, first_solution_time);
                query.bind(6, best_solution_time);
                query.bind(7, total_running_time);
                query.bind(8, init_cost);
                query.bind(9, init_quality);
                query.bind(10, best_cost);
                query.bind(11, best_quality);
                query.bind(12, solution_str);

                try {
                    query.exec();
                } catch (SQLite::Exception &e) {
                    std::cout << "Insert Solution Error\n";
                    std::cout << e.what();
                    return;
                }
            }

        }
    }





}
#endif //TRANSITIONMATRIXROUTE_UTILITY_HPP
