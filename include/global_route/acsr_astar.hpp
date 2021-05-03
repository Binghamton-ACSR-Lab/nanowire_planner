//
// Created by xli185 on 4/15/21.
//

#ifndef TRANSITIONMATRIXROUTE_ACSR_ASTAR_HPP
#define TRANSITIONMATRIXROUTE_ACSR_ASTAR_HPP
#include <chrono>

namespace acsr {
    namespace global {

        class AcsrAstar {
        public:
            /**
             * default constructor
             */
            AcsrAstar() = default;

            /**
             * AcsrAstar
             */
            virtual ~AcsrAstar() = default;

            /**
             * copy constructor deleted
             */
            AcsrAstar(const AcsrAstar &) = delete;

            /**
             * assign operator deleted
             * @return
             */
            AcsrAstar &operator=(const AcsrAstar &) = delete;

            /**
             * \inintialize planner
             * @param n_wire wire count
             * @param init_index init state
             * @param target_index target state
             * @param divided_vec nanowire divided vector
             */
            void init(int n_wire, IndexType init_index, IndexType target_index, const std::vector<int> &divided_vec);

            /**
             * run planner
             */
            void run();

            /**
             * get total running time
             * @return total running time
             */
            long getTotalRunningTime() const;

            /**
             * get first solution time
             * @return first solution time
             */
            long getFirstSolutionTime() const;

            /**
             * get best solution time
             * @return get best solution time
             */
            long getBestSolutionTime() const;

            /**
             * get best solution from target to init
             * @return best solution
             */
            std::vector<IndexType> getBestSolution() const;

            std::vector<IndexType> init_path;
        private:
            ///a vector of control matrix, total size = 4 for nanowire count = 1,2,3,4
            std::vector<ControlMatrixType> _control_matrix_vec;
            ///a vector of transition matrix, total size = 4 for nanowire count = 1,2,3,4
            std::vector<Eigen::SparseMatrix < int, Eigen::ColMajor, IndexType>> _transition_matrix_vec;
            ///wire count
            int _n_wire;
            ///init state
            IndexType _init_index;
            ///target state
            IndexType _target_index;
            ///nanowire divided vector
            std::vector<int> _divided_vec;
            ///running flag
            std::atomic_bool run_flag{false};
            ///a hash map containing nodes to be explored
            std::unordered_map<IndexType, std::shared_ptr<AstarNode>> open_set;
            ///a has map containing nodes been explored
            std::unordered_map<IndexType, std::shared_ptr<AstarNode>> close_set;
            ///best solution data, x: cost, y: quality; both are the smaller the better
            std::pair<int, int> best_solution;
            ///tree root and target node
            std::shared_ptr<AstarNode> root, target;
            ///total running time
            long total_running_time;
            ///first solution running time
            long first_solution_time;
            ///best solution running time
            long best_solution_time;
            ///planner start time point
            std::chrono::time_point<std::chrono::system_clock> start_time_point;
            const int max_running_time = 800; ///seconds


        private:
            /**
             * one propagating step of the planner
             */
            void step();

            /**
             * update the descendants if a node is updated (an updated means a node already exists but get better level or quality)
             * @param node the updated node
             * @param parent the parent of the updated node
             */
            void updateNodeLevel(std::shared_ptr<AstarNode> node, std::shared_ptr<AstarNode> parent);

            /**
             * trim tree
             * @param node the branch root that will be trimmed
             */
            void trimTree(std::shared_ptr<AstarNode> node);

            /**
             * remobe branch
             * @param node branch root
             */
            void removeBranch(std::shared_ptr<TransitionTreeNode> &node);

            /**
             * this fucntion is the same as the one the GlobalRoute, using the transition matrix to get the states of next step
             * @param n_wires
             * @param index
             * @param index2
             * @param divided_vec
             * @param winthin_step
             * @return
             */
            std::vector<IndexType>
            getNextStepIndexVecWithinSteps(int n_wires, IndexType index, IndexType target_index,
                                           const std::vector<int> &divided_vec,
                                           int winthin_step);
        };

        void
        AcsrAstar::init(int n_wire, IndexType init_index, IndexType target_index, const std::vector<int> &divided_vec) {
            ///set up private members
            _n_wire = n_wire;
            _init_index = init_index;
            _target_index = target_index;
            _divided_vec = divided_vec;

            ///read transition matrix data
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
            std::vector<Eigen::SparseMatrix < ControlType, Eigen::ColMajor, IndexType>>
            temp_control_matrix_vec(4);

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

        std::vector<IndexType>
        AcsrAstar::getNextStepIndexVecWithinSteps(int n_wires, IndexType index, IndexType target_index,
                                                  const std::vector<int> &divided_vec, int winthin_step) {
            auto index_vec = indexToSubIndexVec(index, divided_vec);
            auto size = divided_vec.size();
            if (size == 1) {
                std::vector<IndexType> v;
                auto vec = _control_matrix_vec[divided_vec[0] - 1][index_vec[0]].getIndexVector();
                std::copy_if(vec.begin(), vec.end(), std::back_inserter(v),
                             [n_wires, target_index, winthin_step](IndexType i) {
                                 return withinHeuristic(n_wires, i, target_index, winthin_step);
                             });
                return v;
            } else {
                auto target_index_vec = indexToSubIndexVec(target_index, divided_vec);
                std::vector<ControlVectorType> control_vec(size);
                for (auto i = 0; i < size; ++i) {
                    control_vec[i] = _control_matrix_vec[divided_vec[i] - 1][index_vec[i]];
                }
                return controlVectorProductWithStep(control_vec, index_vec, target_index_vec, divided_vec,
                                                    winthin_step);
            }
        }

        void AcsrAstar::run() {
            ///set initial estimate solution steps
            best_solution = {2 + getHeuristic(_n_wire, _init_index, _target_index), 0};
            ///root
            root = std::make_shared<AstarNode>(_init_index, 0, true);
            root->setPathQuality(0);

            open_set[_init_index] = root;

            run_flag = true;
            start_time_point = std::chrono::system_clock::now();

            std::thread t([this]() {
                using namespace std::chrono_literals;
                while (run_flag) {
                    auto time = std::chrono::system_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(time - start_time_point);
                    if (duration.count() > max_running_time) {
                        run_flag = false;
                    }
                    std::this_thread::sleep_for(1s);
                }
            });
            t.detach();

            while (run_flag) {
                step();
            }
            ///get running time
            auto time = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time - start_time_point);
            total_running_time = duration.count();
        }

        void AcsrAstar::step() {
            ///if no node in open set, planner finished
            if (open_set.empty()) {
                run_flag = false;
                return;
            };
            ///get the nearest node
            auto explore_node = *std::min_element(open_set.begin(), open_set.end(),
                                                  [&](const std::pair<IndexType, NodePtr> &n1,
                                                      const std::pair<IndexType, NodePtr> &n2) {
                                                      return getHeuristic(_n_wire, n1.first, _target_index) <
                                                             getHeuristic(_n_wire, n2.first, _target_index);
                                                  });
            ///propagated states
            auto candidate_vec = getNextStepIndexVecWithinSteps(_n_wire, explore_node.first, _target_index,
                                                                _divided_vec,
                                                                best_solution.first - explore_node.second->getLevel() -
                                                                1);
            auto level = explore_node.second->getLevel() + 1;

            for (auto index:candidate_vec) {
                if (index == explore_node.first)continue;
                auto quality = explore_node.second->getPathQuality() + getQuality(_n_wire, index);
                if (open_set.find(index) != open_set.end()) {
                    auto &previous_node = open_set[index];
                    ///if already a node with same state in open set, and the new state has better quality than the previous one
                    if (previous_node->getLevel() > level ||
                        (previous_node->getLevel() == level && previous_node->getPathQuality() > quality)) {
                        previous_node->getParent()->removeChild(previous_node);
                        previous_node->setParent(explore_node.second);
                        explore_node.second->addChild(previous_node);
                        updateNodeLevel(previous_node, explore_node.second);
                    }
                } else if (close_set.find(index) != close_set.end()) {
                    auto &previous_node = close_set[index];
                    ///if already a node with same state in close set, and the new state has better quality than the previous one
                    ///update the previous node, and bring it and it branch to open set
                    if (previous_node->getLevel() > level ||
                        (previous_node->getLevel() == level && previous_node->getPathQuality() > quality)) {
                        previous_node->getParent()->removeChild(previous_node);
                        previous_node->setParent(explore_node.second);
                        explore_node.second->addChild(previous_node);
                        updateNodeLevel(previous_node, explore_node.second);
                    }
                } else {
                    ///no node with same state exists, create new node and put it to open set
                    auto n = std::make_shared<AstarNode>(index, level, true);
                    n->setParent(explore_node.second);
                    explore_node.second->addChild(n);
                    n->setPathQuality(quality);
                    open_set[index] = n;
                }
                ///first solution
                if (target == nullptr && index == _target_index) {
                    target = open_set[index];
                    open_set.erase(index);
                    ///put the target to close set due to no need to explore
                    close_set.insert(std::make_pair(index, target));
                    target->setNodeState(false);
                    best_solution.first = target->getLevel();
                    best_solution.second = target->getPathQuality();
                    ///save first solution time
                    auto time = std::chrono::system_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time - start_time_point);
                    first_solution_time = duration.count();
                    best_solution_time = first_solution_time;

                    init_path = getBestSolution();
                    ///trim tree, remove the nodes with greater cost than target
                    trimTree(root);
                }

                ///find target state again
                if (target != nullptr &&
                    (target->getLevel() != best_solution.first || target->getPathQuality() != best_solution.second)) {
                    best_solution.first = target->getLevel();
                    best_solution.second = target->getPathQuality();
                    ///update the best solution time
                    auto time = std::chrono::system_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time - start_time_point);
                    best_solution_time = duration.count();
                    ///trim tree, remove the nodes with greater cost than target
                    trimTree(root);
                }
            }
            ///put the explored node to close set
            open_set.erase(explore_node.first);
            close_set[explore_node.first] = explore_node.second;
            explore_node.second->setNodeState(false);
        }

        void AcsrAstar::updateNodeLevel(std::shared_ptr<AstarNode> node, std::shared_ptr<AstarNode> parent) {

            if (!node->getNodeState() || node != target) {
                close_set.erase(node->getState());
                open_set[node->getState()] = node;
                node->setNodeState(true);
            }
            node->setPathQuality(parent->getPathQuality() + getQuality(_n_wire, node->getState()));
            node->setLevel(parent->getLevel() + 1);
            for (auto &it:node->getChildren()) {
                updateNodeLevel(std::static_pointer_cast<AstarNode>(it), node);
            }
        }

        void AcsrAstar::trimTree(std::shared_ptr<AstarNode> node) {
            ///working on children
            for (auto it = node->getChildren().begin(); it != node->getChildren().end();) {
                if ((*it) == target) {
                    ++it;
                    continue;
                }
                ///if the child not better than target, remove this child branch
                if ((*it)->getLevel() + getHeuristic(_n_wire, (*it)->getState(), _target_index) > target->getLevel()) {
                    for (auto &child:(*it)->getChildren()) {
                        removeBranch(child);
                    }
                    ///the first call of removeBranch (not recursive) will not remove the node nor clear its children, you have to call this two procedure manually
                    (*it)->getChildren().clear();
                    it = node->getChildren().erase(it);
                } else { ///if not, working on the children of the child
                    trimTree(std::static_pointer_cast<AstarNode>(*it));
                    ++it;
                }
            }
        }

        void AcsrAstar::removeBranch(std::shared_ptr<TransitionTreeNode> &node) {
            ///remove from open set or close set
            if (std::static_pointer_cast<AstarNode>(node)->getNodeState()) {
                open_set.erase(node->getState());
            } else {
                close_set.erase(node->getState());
            }
            node->setParent(nullptr);
            ///remove its children
            for (auto it:node->getChildren()) {
                removeBranch(it);
            }
            node->getChildren().clear();
        }

        long AcsrAstar::getFirstSolutionTime() const {
            return first_solution_time;
        }

        long AcsrAstar::getBestSolutionTime() const {
            return best_solution_time;
        }

        long AcsrAstar::getTotalRunningTime() const {
            return total_running_time;
        }

        std::vector<IndexType> AcsrAstar::getBestSolution() const {
            std::vector<IndexType> solution;
            if (root == nullptr || target == nullptr)return solution;
            auto n = target;
            solution.push_back(n->getState());
            while (n->getLevel() > 0) {
                n = std::static_pointer_cast<AstarNode>(n->getParent());
                solution.push_back(n->getState());
            }
            return solution;
        }
    }

}

#endif //TRANSITIONMATRIXROUTE_ACSR_ASTAR_HPP
