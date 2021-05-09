//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_PLANNER_HPP
#define NANOWIREPLANNER_NANOWIRE_PLANNER_HPP
#include <utility>
#include <numeric>
#include <random>
#include "nanowire_system.hpp"
#include "nanowire_utility.hpp"
#include "observer/observer.hpp"

namespace acsr {
    struct PlannerConnection{
    public:

        PlannerConnection() = delete;
        PlannerConnection(std::pair<TreeNodePtr,TreeNodePtr> end_states,
                          std::vector<Eigen::VectorXd>& state,
                          std::vector<Eigen::VectorXd>& controls,
                          std::vector<double>& durations):
                _states(std::move(state)),_controls(std::move(controls)),_durations(std::move(durations)),_end_states(std::move(end_states))
        {

        }
        bool isEmpty(){
            return _durations.empty();
        }

        double getTotalCost(){
            return std::accumulate(_durations.begin(),_durations.end(),0.0);
        }

        std::vector<Eigen::VectorXd> _states{};
        std::vector<Eigen::VectorXd> _controls{};
        std::vector<double> _durations;
        std::pair<TreeNodePtr,TreeNodePtr> _end_states;
    };

    class Planner {

    protected:
        std::shared_ptr<NanowireSystem> _dynamic_system;
        std::shared_ptr<PlannerConnection> _planner_connection;

        Eigen::VectorXd _init_state;
        Eigen::VectorXd _target_state;

        double _goal_radius;

        std::pair<TreeNodePtr, TreeNodePtr> _best_goal;
        std::pair<unsigned long, unsigned long> _number_of_nodes;

        bool _is_bi_tree_planner = false;
        bool _is_optimized_connect = false;



        std::vector<std::shared_ptr<SolutionUpdateObserver>> solution_update_observers;
        std::vector<std::shared_ptr<PlannerStartObserver>> planner_start_observers;
        std::vector<std::shared_ptr<NodeAddedObserver>> node_added_observers;

        /// a flag to indicate the planner is stopped. This flag is used to terminate long time process
        std::atomic_bool _run_flag;


        /***
        *
        * @param node: a treenode
        * @return check current best path going through node
        */
        bool isInSolutionPath(TreeNodePtr node) {
            if (_best_goal.first == nullptr) return false;
            if (node->getTreeId() == TreeId::forward) {
                auto it = _best_goal.first;
                while (it) {
                    if (it == node)return true;
                    it = it->getParent();
                }
            } else if (node->getTreeId() == TreeId::reverse) {
                auto it = _best_goal.second;
                while (it) {
                    if (it == node)return true;
                    it = it->getParent();
                }
            }
            return false;
        }






    public:
        Planner() = delete;

        explicit Planner(std::shared_ptr<NanowireSystem> system):_dynamic_system(system), _run_flag(false), _goal_radius(0.0) {
        }

        virtual ~Planner() = default;

        /***
         * necessary setups when starting a planner
         */
        virtual void setup(const std::shared_ptr<NanowireConfig>& nanowire_config) = 0;

        /***
         * one forward step. this function can be called in a while loop for a forward propagating process
         */
        virtual void forwardStep() = 0;

        /***
         * one reverse step. this function can be called in a while loop for a reverse propagating process
         */
        virtual void reverseStep() = 0;

        /***
         * one connecting step. this function can be called in a while loop for a connecting process
         */
        virtual void connectingStep() = 0;

        /***
         * get the current numbers of nodes
         * @return a pair of nodes on forward tree and nodes on reverse tree
         */
        virtual std::pair<unsigned long, unsigned long> getNumberOfNode() {
            return _number_of_nodes;
        }

        /***
         *
         * @return goal radius
         */
        virtual double getGoalRadius() const {
            return _goal_radius;
        }

        /***
         * set goal radius
         */
        virtual void setGoalRadius(double goal_radius) {
            _goal_radius = goal_radius;
        }

        /***
         * get start state
         * @return
         */
        virtual Eigen::VectorXd getStartState() const {
            return _init_state;
        }

        /***
         * set start state
         */
        virtual void setStartState(const Eigen::VectorXd &init_state) {
            _init_state = init_state;
        }

        /***
         * get target state
         * @return
         */
        virtual Eigen::VectorXd getTargetState() const {
            return _target_state;
        }

        /***
         * set target state
         */
        virtual void setTargetState(const Eigen::VectorXd &target_state) {
            _target_state = target_state;
        }

        /***
         * check whether the planner is bi-directional explore, e.g.the reverse propagating process is employed
         * @return
         */
        virtual bool isBiTreePlanner() const {
            return _is_bi_tree_planner;
        }

        /***
         * set whether the planner is bi-directional explore, e.g.the reverse propagating process is employed
         */
        virtual void setBiTreePlanner(bool bi_tree_planner) {
            _is_bi_tree_planner = bi_tree_planner;
        }

        /***
         * check whether the connecting process is empplyed
         * @return
         */
        virtual bool isOptimizedConnect() const {
            return _is_optimized_connect;
        }

        /***
         * set whether the connecting process is empplyed
         */
        virtual void setOptimizedConnect(bool optimized_connect) {
            _is_optimized_connect = optimized_connect;
        }



        /***
         * register a solution update observer
         * @param observer inherits from SolutionUpdateObserver
         */

       virtual void registerSolutionUpdateObserver(const std::shared_ptr<SolutionUpdateObserver>& observer){
           this->solution_update_observers.push_back(observer);
       }

        /***
         * unregister a solution update observer
         * @param observer inherits from SolutionUpdateObserver
         */

       virtual void unregisterSolutionUpdateObserver(const std::shared_ptr<SolutionUpdateObserver>& observer){
           this->solution_update_observers.erase(std::find(solution_update_observers.begin(),solution_update_observers.end(),observer));
       }

        /***
          * register a node added observer
          * @param observer inherits from NodeAddedObserver
          */

        virtual void registerNodeAddedObserver(const std::shared_ptr<NodeAddedObserver>& observer){
            node_added_observers.push_back(observer);
        }

        /***
         * unregister a node added observer
         * @param observer inherits from NodeAddedObserver
         */

        virtual void unregisterSolutionUpdateObserver(const std::shared_ptr<NodeAddedObserver>& observer){
            node_added_observers.erase(std::find(node_added_observers.begin(),node_added_observers.end(),observer));
        }



        /***
         * register a planner started observer
         * @param observer inherits from PlannerStartObserver
         */

       virtual void registerPlannerStartObserver(const std::shared_ptr<PlannerStartObserver>& observer){
           this->planner_start_observers.push_back(observer);
       }

        /***
         * unregister a planner started observer
         * @param observer inherits from PlannerStartObserver
         */

       virtual void unregisterPlannerStartObserver(const std::shared_ptr<PlannerStartObserver>& observer){
           this->planner_start_observers.erase(std::find(planner_start_observers.begin(),planner_start_observers.end(),observer));
       }

        /***
         * get the solution data for solution update observers.
         * @param forward_states
         * @param reverse_states
         * @param connect_states
         * @param forward_control
         * @param reverse_control
         * @param connect_control
         * @param forward_durations
         * @param reverse_durations
         * @param connect_durations
         */
        void getSolutionVectors(std::vector<Eigen::VectorXd> &forward_states,
                                std::vector<Eigen::VectorXd> &reverse_states,
                                std::vector<Eigen::VectorXd> &connect_states,
                                std::vector<Eigen::VectorXd> &forward_control,
                                std::vector<Eigen::VectorXd> &reverse_control,
                                std::vector<Eigen::VectorXd> &connect_control,
                                std::vector<double> &forward_durations,
                                std::vector<double> &reverse_durations,
                                std::vector<double> &connect_durations) {
            forward_states.clear();
            reverse_states.clear();
            connect_states.clear();
            forward_control.clear();
            reverse_control.clear();
            connect_control.clear();
            forward_durations.clear();
            reverse_durations.clear();
            connect_durations.clear();

            if (this->_best_goal.first == nullptr) {
                return;
            }
            auto node = this->_best_goal.first;
            while (node) {
                forward_states.push_back(node->getState());
                forward_control.push_back(node->getEdgeControl());
                forward_durations.push_back(node->getEdgeDuration());
                node = node->getParent();
            }

            node = _best_goal.second;
            reverse_durations.push_back(0);
            reverse_control.push_back(Eigen::VectorXd(_dynamic_system->getControlDimension()));
            while (node) {
                reverse_states.push_back(node->getState());
                reverse_control.push_back(node->getEdgeControl());
                reverse_durations.push_back(node->getEdgeDuration());
                node = node->getParent();
            }
            reverse_durations.pop_back();
            reverse_control.pop_back();

            if (_planner_connection != nullptr && _planner_connection->_end_states == _best_goal) {
                connect_states = _planner_connection->_states;
                connect_control = _planner_connection->_controls;
                connect_durations = _planner_connection->_durations;
            }

            std::reverse(forward_states.begin(), forward_states.end());
            std::reverse(forward_control.begin(), forward_control.end());
            std::reverse(forward_durations.begin(), forward_durations.end());
        }


        /***
         *
         * @return the cost of current path. return the max value of double type if no path exists
         */
        virtual double getMaxCost() {
            if (_best_goal.first == nullptr || _best_goal.second == nullptr)
                return std::numeric_limits<double>::max();
            double cost = _best_goal.first->getCost() + _best_goal.second->getCost();
            if (_planner_connection != nullptr && _planner_connection->_end_states.first == _best_goal.first
                && _planner_connection->_end_states.second == _best_goal.second) {
                cost += _planner_connection->getTotalCost();
            }
            return cost;
        }

        /***
         * notify the planner to stop. here the run_flag can be set to false. Also can call a function to stop the process in dynamic system
         */
        virtual void stop() {
            _run_flag = false;
            _dynamic_system->stop();
        }


        /***
             * notify planner start observers. This function should be manually called when starting a planner
             */

       virtual void notifyPlannerStart(const std::string& planner_name,const std::string& image_name,const VariablesGrid& reference) {
           for(auto observer: planner_start_observers){
               observer->onPlannerStart(planner_name,_dynamic_system->getRobotCount(),_init_state,_target_state,reference,Config::bidirection,Config::optimization,
                                        Config::total_time,Config::goal_radius,Config::integration_step,Config::min_time_steps,Config::max_time_steps,
                                        Config::sst_delta_near,Config::sst_delta_drain,Config::optimization_distance,Config::blossomM,Config::blossomN,Config::dominant_path_count,Config::quality_decrease_factor,image_name);
           }
       }

        virtual void notifyNodeAdded(const Eigen::VectorXd& state,TreeId id) {
           for(auto observer: node_added_observers){
               observer->onNodeAdded(state,id);
           }
        }

        /***
  * notify solution update observers. This function should be manually called when a solution is updated
  */

        virtual void notifySolutionUpdate() {
            std::vector<Eigen::VectorXd> forward_state;
            std::vector<Eigen::VectorXd> reverse_state;
            std::vector<Eigen::VectorXd> connect_state;
            std::vector<Eigen::VectorXd> forward_control;
            std::vector<Eigen::VectorXd> reverse_control;
            std::vector<Eigen::VectorXd> connect_control;
            std::vector<double> forward_durations;
            std::vector<double> reverse_durations;
            std::vector<double> connect_durations;
            getSolutionVectors(forward_state,reverse_state,connect_state,
                               forward_control,reverse_control,connect_control,
                               forward_durations,reverse_durations,connect_durations);

            if(!_run_flag)return;
            for(auto observer: solution_update_observers){
                observer->onSolutionUpdate(forward_state,reverse_state,connect_state,forward_control,reverse_control,connect_control,forward_durations,reverse_durations,connect_durations);
            }
        }
    };
}


#endif //NANOWIREPLANNER_NANOWIRE_PLANNER_HPP
