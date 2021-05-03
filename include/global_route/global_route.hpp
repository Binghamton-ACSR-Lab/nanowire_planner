//
// Created by acsr on 4/5/21.
//

#ifndef TRANSITIONMATRIXROUTE_GLOBAL_ROUTE_HPP
#define TRANSITIONMATRIXROUTE_GLOBAL_ROUTE_HPP

#include <unordered_set>
#include <thread>
#include "node.hpp"
#include "utility.hpp"
#include "transition_matrix_constructor.hpp"
#include "boost/filesystem.hpp"
#include <mutex>
#include <acado/external_packages/eigen3/Eigen/Eigen>
//#include <SQLiteCpp/SQLiteCpp.h>
#include <future>
#include <set>

namespace acsr {
    namespace global {
        using ControlMatrixType = std::vector<ControlVectorType>;
        //USING_NAMESPACE_ACADO;


        class GlobalRoute {
        public:
            /**
             * default constructor
             */
            GlobalRoute() = default;

            /**
             * deconstructor
             */
            virtual ~GlobalRoute();

            /**
             * copy constructor deleted
             */
            GlobalRoute(const GlobalRoute &) = delete;

            /**
             * assign operator deleted
             * @return
             */
            GlobalRoute &operator=(const GlobalRoute &) = delete;

            /**
             * initialize
             */
            void init();

            /**
             * construct a tree
             * @param n_wire nanowire count
             * @param init_index init index corresponding state
             * @param target_index target index corresponding state
             * @param divided_vec a vector to divide n_wire to small count
             * @return true if construct tree success
             */
            bool constructTree(int n_wire, IndexType init_index, IndexType target_index,
                               const std::vector<int> &divided_vec);

            /**
             * get best solution
             * @return a vector of position index from target to init
             */
            std::vector<IndexType> getBestSolution();

            /**
            * get the posible indice of performing one step from index
            * @param index the original index
            * @param divided_vec a vector of nanowire count
            * @return the vector of possible indice
            */
            std::vector<IndexType>
            getNextStepIndexVec(int n_wire, IndexType index, const std::vector<int> &divided_vec);


        private:
            ///a vector of control matrix, total size = 4 for nanowire count = 1,2,3,4
            std::vector<ControlMatrixType> _control_matrix_vec;
            ///a vector of transition matrix, total size = 4 for nanowire count = 1,2,3,4
            std::vector<Eigen::SparseMatrix<int, Eigen::ColMajor, IndexType>> _transition_matrix_vec;
            ///the root of the constructed tree
            NodePtr root = nullptr;
            ///target of the constructed tree
            NodePtr target = nullptr;
            /// astar running flag



        private:

            /**
             * get the posible indice of performing one step from index, the cost of those indice to target should within step
             * @param n_wires wires count
             * @param index the original index
             * @param target target
             * @param divided_vec a vector of nanowire count
             * @param step max cost to target
             * @return the vector of possible indice
             */
            std::vector<IndexType> getNextStepIndexVecWithinSteps(int n_wires, IndexType index, IndexType target,
                                                                  const std::vector<int> &divided_vec, int step);

            /**
             * combine two hash map
             * @param m1 map1
             * @param m2 map2
             */
            void combineMap(std::unordered_map<IndexType, NodePtr> &m1, std::unordered_map<IndexType, NodePtr> &m2);
        };

        void GlobalRoute::init() {
            TransitionMatrixConstructor constructor;
            using namespace boost::filesystem;

            ///construct transition matrix if not exist
            if (!exists("data") || !exists("data/transition_matrix")) {
                create_directories("data/transition_matrix");
            }
            if (!exists("data/transition_matrix/_matrix_1_.bin")) {
                constructor.constructMatrix(1, "data/transition_matrix/_matrix_1_.bin");
            }
            if (!exists("data/transition_matrix/_matrix_2_.bin")) {
                constructor.constructMatrix(2, "data/transition_matrix/_matrix_2_.bin");
            }
            if (!exists("data/transition_matrix/_matrix_3_.bin")) {
                constructor.constructMatrix(3, "data/transition_matrix/_matrix_3_.bin");
            }
            if (!exists("data/transition_matrix/_matrix_4_.bin")) {
                constructor.constructMatrix(4, "data/transition_matrix/_matrix_4_.bin");
            }

            ///construct control matrix if not exist
            if (!exists("data/transition_matrix/_control_matrix_1_.bin")) {
                constructor.constructControlMatrix(1, "data/transition_matrix/_control_matrix_1_.bin");
            }
            if (!exists("data/transition_matrix/_control_matrix_2_.bin")) {
                constructor.constructControlMatrix(2, "data/transition_matrix/_control_matrix_2_.bin");
            }
            if (!exists("data/transition_matrix/_control_matrix_3_.bin")) {
                constructor.constructControlMatrix(3, "data/transition_matrix/_control_matrix_3_.bin");
            }
            if (!exists("data/transition_matrix/_control_matrix_4_.bin")) {
                constructor.constructControlMatrix(4, "data/transition_matrix/_control_matrix_4_.bin");
            }
            std::vector<Eigen::SparseMatrix<ControlType, Eigen::ColMajor, IndexType>> temp_control_matrix_vec(4);

            _transition_matrix_vec.resize(4);
            _control_matrix_vec.resize(4);

            ///read data from file
            for (auto i = 0; i < 4; ++i) {
                char ss1[200], ss2[200];
                sprintf(ss1, "data/transition_matrix/_matrix_%d_.bin", i + 1);
                sprintf(ss2, "data/transition_matrix/_control_matrix_%d_.bin", i + 1);
                TransitionMatrixConstructor::readTransitionMatrix(ss1, _transition_matrix_vec[i]);
                TransitionMatrixConstructor::readControlMatrix(ss2, temp_control_matrix_vec[i]);
                _control_matrix_vec[i].resize(temp_control_matrix_vec[i].cols());
                std::vector<std::vector<std::pair<IndexType, ControlType>>> data(temp_control_matrix_vec[i].cols());
                for (int k = 0; k < temp_control_matrix_vec[i].outerSize(); ++k) {
                    for (Eigen::SparseMatrix<ControlType, Eigen::ColMajor, IndexType>::InnerIterator it(
                            temp_control_matrix_vec[i], k); it; ++it) {
                        data[it.col()].push_back(std::make_pair(it.row(), it.value()));
                    }
                }
                for (auto k = 0; k < temp_control_matrix_vec[i].cols(); ++k) {
                    _control_matrix_vec[i][k].setSize(temp_control_matrix_vec[i].rows());
                    _control_matrix_vec[i][k].setData(std::move(data[k]));
                }
            }
            std::cout << "Initialize Complete\n";
        }

        /**
         * construct a tree
         * @param n_wire nanowire count
         * @param init_index init index corresponding state
         * @param target_index target index corresponding state
         * @param divided_vec a vector to divide n_wire to small count
         * @return true if construct tree success
         */
        bool GlobalRoute::constructTree(int n_wire, IndexType init_index, IndexType target_index,
                                        const std::vector<int> &divided_vec) {
            if (init_index == target_index) {
                std::cout << "Start and Target States Are the Same!\n";
                return false;
            }
            IndexType dimension = 1;
            for (auto i = 0; i < n_wire; ++i)
                dimension = dimension * 16;
            auto processor = std::thread::hardware_concurrency();
            for (auto over_step = 0; over_step < 2; ++over_step) {
                auto step = getHeuristic(n_wire, init_index, target_index) + over_step;
                target = nullptr;
                root = std::make_shared<TransitionTreeNode>(init_index, 0);
                root->setPathQuality(0);
                std::vector parent_vec{root};
                for (auto i = 0; i < step; ++i) {
                    if (parent_vec.empty()) {
                        root.reset();
                        break;
                    }
                    auto total_size = parent_vec.size();
                    auto thread_count = std::min(int(processor), int(total_size));
                    auto count = total_size / thread_count;
                    std::vector<std::unordered_map<IndexType, NodePtr>> children_map_vecs(thread_count);
                    std::vector<std::thread> thread_vec;
                    std::cout << "Construct Tree, Step " << i + 1 << "\n";
                    //auto start = std::chrono::high_resolution_clock::now();
                    for (auto k = 0; k < thread_count - 1; ++k) {
                        auto &m = children_map_vecs[k];
                        thread_vec.emplace_back(
                                [target_index, n_wire, i, step, &parent_vec, count, k, &m, this, &divided_vec]() {
                                    for (auto index = k * count; index < (k + 1) * count; ++index) {
                                        auto current_quality = parent_vec[index]->getPathQuality();
                                        auto current_parent = parent_vec[index];
                                        auto vec = getNextStepIndexVecWithinSteps(n_wire, current_parent->getState(),
                                                                                  target_index, divided_vec,
                                                                                  step - i - 1);

                                        for (auto &v:vec) {
                                            if (v == current_parent->getState())continue;
                                            auto &p = m[v];
                                            if (p == nullptr || p->getPathQuality() > current_quality) {
                                                p = current_parent;
                                            }
                                        }
                                    }
                                });
                    }

                    auto &m = children_map_vecs[thread_count - 1];
                    for (auto index = count * (thread_count - 1); index < total_size; ++index) {
                        auto current_quality = parent_vec[index]->getPathQuality();
                        auto current_parent = parent_vec[index];
                        std::vector<IndexType> vec = getNextStepIndexVecWithinSteps(n_wire, current_parent->getState(),
                                                                                    target_index, divided_vec,
                                                                                    step - i - 1);
                        for (auto &v:vec) {
                            if (v == current_parent->getState())continue;
                            auto &p = m[v];
                            if (p == nullptr || p->getPathQuality() > current_quality) {
                                p = current_parent;
                            }
                        }
                    }

                    for (auto &t:thread_vec) {
                        if (t.joinable())t.join();
                    }

//                auto stop = std::chrono::high_resolution_clock::now();
//                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
//                std::cout << "Find Layer Cost: " << duration.count() << '\n';
//                start = stop;


                    while (thread_count > 1) {
                        auto remain = thread_count % 2;
                        int new_thread_count = thread_count / 2;
                        std::vector<std::thread> combine_thread_vec;
                        for (auto i = 0; i < new_thread_count - 1; ++i) {
                            combine_thread_vec.emplace_back(&GlobalRoute::combineMap, this,
                                                            std::ref(children_map_vecs[i]),
                                                            std::ref(children_map_vecs[thread_count - 1 - i]));
                        }
                        combineMap(children_map_vecs[new_thread_count - 1],
                                   children_map_vecs[thread_count - new_thread_count]);
                        for (auto &t:combine_thread_vec) {
                            if (t.joinable())t.join();
                        }
                        thread_count = new_thread_count + remain;
                    }
                    auto children_map = std::move(children_map_vecs[0]);

//                stop = std::chrono::high_resolution_clock::now();
//                duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
//                std::cout << "Combine Cost: " << duration.count() << '\n';
//                start = stop;

                    std::vector<NodePtr> temp_parent_vec;
                    for (auto &p:children_map) {
                        auto node = std::make_shared<TransitionTreeNode>(p.first, i + 1);
                        node->setPathQuality(p.second->getPathQuality() + getQuality(n_wire, p.first));
                        node->setParent(p.second);
                        p.second->addChild(node);
                        temp_parent_vec.push_back(node);
                    }
                    if (i == step - 1 && !temp_parent_vec.empty()) {
                        target = temp_parent_vec[0];
                    }
//                stop = std::chrono::high_resolution_clock::now();
//                duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
//                std::cout << "Add Node Cost: " << duration.count() << '\n';
                    /*
                    for (auto &p:parent_vec) {
                        trimLeaf(p);
                    }
                     */
                    parent_vec = std::move(temp_parent_vec);
                }
                if (target == nullptr) {
                    root.reset();
                } else {
                    std::cout << "Construct Forward Tree Successfully!\n";
                    return true;
                }
            }
            return false;

        }

        /**
         * deconstruct, destroy tree
         */
        GlobalRoute::~GlobalRoute() {
            root.reset();
        }

        /**
         * get best solution
         * @return best solution, from target to start state
         */
        std::vector<IndexType> GlobalRoute::getBestSolution() {
            std::vector<IndexType> solution;
            if (root == nullptr || target == nullptr)return solution;
            auto n = target;
            solution.push_back(n->getState());
            while (n->getLevel() >= 1) {
                auto p = n->getParent();
                solution.push_back(p->getState());
                n = p;
            }
            return solution;
        }

        /**
         * get the posible indice of performing one step from index, the cost of those indice to target should within step
         * @param n_wires wires count
         * @param index the original index
         * @param target target
         * @param divided_vec a vector of nanowire count
         * @param step max cost to target
         * @return the vector of possible indice
         */
        std::vector<IndexType>
        GlobalRoute::getNextStepIndexVecWithinSteps(int n_wires, IndexType index, IndexType index2,
                                                    const std::vector<int> &divided_vec, int winthin_step) {
            auto index_vec = indexToSubIndexVec(index, divided_vec);
            auto size = divided_vec.size();
            if (size == 1) {
                std::vector<IndexType> v;
                auto vec = _control_matrix_vec[divided_vec[0] - 1][index_vec[0]].getIndexVector();
                std::copy_if(vec.begin(), vec.end(), std::back_inserter(v),
                             [n_wires, index2, winthin_step](IndexType i) {
                                 return withinHeuristic(n_wires, i, index2, winthin_step);
                             });
                return v;
            } else {
                auto target_index_vec = indexToSubIndexVec(index2, divided_vec);
                std::vector<ControlVectorType> control_vec(size);
                for (auto i = 0; i < size; ++i) {
                    control_vec[i] = _control_matrix_vec[divided_vec[i] - 1][index_vec[i]];
                }
                return controlVectorProductWithStep(control_vec, index_vec, target_index_vec, divided_vec,
                                                    winthin_step);
            }
        }

        /**
         * combine two hash map
         * @param m1 map1
         * @param m2 map2
         */
        void GlobalRoute::combineMap(std::unordered_map<IndexType, NodePtr> &map1,
                                     std::unordered_map<IndexType, NodePtr> &map2) {
            if (map1.empty() && map2.empty())return;
            for (auto &it : map2) {
                auto &exist_pair = map1[it.first];
                if (exist_pair == nullptr || exist_pair->getPathQuality() > it.second->getPathQuality()) {
                    exist_pair = std::move(it.second);
                }
            }
            map2.clear();
        }

        std::vector<IndexType>
        GlobalRoute::getNextStepIndexVec(int n_wire, IndexType index, const std::vector<int> &divided_vec) {
            auto index_vec = indexToSubIndexVec(index, divided_vec);
            auto size = divided_vec.size();
            if (size == 1) {
                std::vector<IndexType> v;
                return _control_matrix_vec[divided_vec[0] - 1][index_vec[0]].getIndexVector();
            } else {
                std::vector<ControlVectorType> control_vec(size);
                for (auto i = 0; i < size; ++i) {
                    control_vec[i] = _control_matrix_vec[divided_vec[i] - 1][index_vec[i]];
                }
                return controlVectorProduct(control_vec, index_vec, divided_vec);
            }
        }
    }
}
#endif //TRANSITIONMATRIXROUTE_GLOBAL_ROUTE_HPP
