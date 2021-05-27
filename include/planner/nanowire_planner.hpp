//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_PLANNER_HPP
#define NANOWIREPLANNER_NANOWIRE_PLANNER_HPP
#include <utility>
#include <numeric>
#include <random>
#include <config/system_config.hpp>
#include "nanowire_system.hpp"
#include "nanowire_utility.hpp"
#include "observer/observer.hpp"

namespace acsr {
    /***
     * struct to storing connecting segment information
     */
     template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    struct PlannerConnection{
        using StateType = Eigen::Matrix<double,STATE_DIMENSION,1>;
        using ControlType = Eigen::Matrix<double,CONTROL_DIMENSION,1>;
        using NodePtr = std::shared_ptr<Node<STATE_DIMENSION>>;
        using TreeNodePtr = std::shared_ptr <TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>>;
    public:

        /***
         * default construct
         */
        PlannerConnection() = delete;

        /***
         * construct
         * @param end_states two ends states from forward and backward tree
         * @param state a vector to store intermedia states from beginning states to target states
         * @param controls a vector to store intermedia controls from beginning states to target states
         * @param durations a vector to store intermedia duration from beginning states to target states
         */
        PlannerConnection(std::pair<TreeNodePtr,TreeNodePtr> end_states,
                          std::vector<StateType>& state,
                          std::vector<ControlType>& controls,
                          std::vector<double>& durations):
                _states(std::move(state)),_controls(std::move(controls)),_durations(std::move(durations)),_end_states(std::move(end_states))
        {
        }

        /***
         * check if no data
         * @return
         */
        bool isEmpty(){
            return _durations.empty();
        }

        /***
         * get total cost of this connection segment
         * @return
         */
        double getTotalCost(){
            return std::accumulate(_durations.begin(),_durations.end(),0.0);
        }

        std::vector<StateType> _states{};
        std::vector<ControlType> _controls{};
        std::vector<double> _durations;
        std::pair<TreeNodePtr,TreeNodePtr> _end_states;
    };

    /***
     * abstract class for planner
     */
     template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class Planner {
        using StateType = Eigen::Matrix<double,STATE_DIMENSION,1>;
        using ControlType = Eigen::Matrix<double,CONTROL_DIMENSION,1>;;
        using NodePtr = std::shared_ptr<Node<STATE_DIMENSION>>;
        using TreeNodePtr = std::shared_ptr <TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>>;
    protected:
        //const int _state_dimension;
        std::shared_ptr<NanowireSystem<STATE_DIMENSION/2,CONTROL_DIMENSION>> _dynamic_system; ///nanowire system
        std::shared_ptr<PlannerConnection<STATE_DIMENSION,CONTROL_DIMENSION>> _planner_connection;/// connecting segment

        StateType _init_state;///start state
        StateType _target_state;///target state

        double _goal_radius;///goal radius

        double _best_cost = std::numeric_limits<double>::max();
        std::pair<TreeNodePtr, TreeNodePtr> _best_goal;///best goals
        std::pair<unsigned long, unsigned long> _number_of_nodes;///nodes on kdtree
        bool _is_bi_tree_planner = false; ///flag to indicate bi-tree
        bool _is_optimized_connect = false;///flag to indicate adopting BVP to connect two trees

        ///observers
        std::vector<std::shared_ptr<SolutionUpdateObserver<STATE_DIMENSION,CONTROL_DIMENSION>>> solution_update_observers;
        std::vector<std::shared_ptr<PlannerStartObserver<STATE_DIMENSION,CONTROL_DIMENSION>>> planner_start_observers;
        std::vector<std::shared_ptr<NodeAddedObserver<STATE_DIMENSION,CONTROL_DIMENSION>>> node_added_observers;

        /// a flag to indicate the planner is stopped. This flag is used to terminate long time process
        std::atomic_bool _run_flag;

        /***
         * check whether a node is in path
         * @param node
         * @return
         */
        bool isInSolutionPath(TreeNodePtr node) {
            if (_best_goal.first == nullptr) return false;
            if (node->getTreeId() == TreeId::forward) {
                auto it = _best_goal.first;
                while (it) {
                    if (it == node)return true;
                    it = it->getParent();
                }
            } else if (node->getTreeId() == TreeId::backward) {
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

        /***
         * construct
         * @param system
         */
        explicit Planner(std::shared_ptr<NanowireSystem<STATE_DIMENSION/2,CONTROL_DIMENSION>> system):_dynamic_system(system),
            _run_flag(false),
            _goal_radius(0.0){

        }

        virtual ~Planner() = default;

        /***
         * necessary setups when starting a planner
         */
        virtual void setup() = 0;

        /***
         * one forward step. this function can be called in a while loop for a forward propagating process
         */
        virtual void forwardStep() = 0;

        /***
         * one backward step. this function can be called in a while loop for a backward propagating process
         */
        virtual void backwardStep() = 0;

        /***
         * one connecting step. this function can be called in a while loop for a connecting process
         */
        virtual void connectingStep() = 0;

        /***
         * get the current numbers of nodes
         * @return a pair of nodes on forward tree and nodes on backward tree
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
        virtual StateType getStartState() const {
            return _init_state;
        }

        /***
         * set start state
         */
        virtual void setStartState(const StateType &init_state) {
            _init_state = init_state;
        }

        /***
         * get target state
         * @return
         */
        virtual StateType getTargetState() const {
            return _target_state;
        }

        /***
         * set target state
         */
        virtual void setTargetState(const StateType &target_state) {
            _target_state = target_state;
        }

        /***
         * check whether the planner is bi-directional explore, e.g.the backward propagating process is employed
         * @return
         */
        virtual bool isBiTreePlanner() const {
            return _is_bi_tree_planner;
        }

        /***
         * set whether the planner is bi-directional explore, e.g.the backward propagating process is employed
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

        virtual StateType forward(const StateType& state,const ControlType& controls,double duration) = 0;



        /***
         * register a solution update observer
         * @param observer inherits from SolutionUpdateObserver
         */

       virtual void registerSolutionUpdateObserver(const std::shared_ptr<SolutionUpdateObserver<STATE_DIMENSION,CONTROL_DIMENSION>>& observer){
           this->solution_update_observers.push_back(observer);
       }

        /***
         * unregister a solution update observer
         * @param observer inherits from SolutionUpdateObserver
         */

       virtual void unregisterSolutionUpdateObserver(const std::shared_ptr<SolutionUpdateObserver<STATE_DIMENSION,CONTROL_DIMENSION>>& observer){
           this->solution_update_observers.erase(std::find(solution_update_observers.begin(),solution_update_observers.end(),observer));
       }

        /***
          * register a node added observer
          * @param observer inherits from NodeAddedObserver
          */

        virtual void registerNodeAddedObserver(const std::shared_ptr<NodeAddedObserver<STATE_DIMENSION,CONTROL_DIMENSION>>& observer){
            node_added_observers.push_back(observer);
        }

        /***
         * unregister a node added observer
         * @param observer inherits from NodeAddedObserver
         */

        virtual void unregisterSolutionUpdateObserver(const std::shared_ptr<NodeAddedObserver<STATE_DIMENSION,CONTROL_DIMENSION>>& observer){
            node_added_observers.erase(std::find(node_added_observers.begin(),node_added_observers.end(),observer));
        }



        /***
         * register a planner started observer
         * @param observer inherits from PlannerStartObserver
         */

       virtual void registerPlannerStartObserver(const std::shared_ptr<PlannerStartObserver<STATE_DIMENSION,CONTROL_DIMENSION>>& observer){
           this->planner_start_observers.push_back(observer);
       }

        /***
         * unregister a planner started observer
         * @param observer inherits from PlannerStartObserver
         */

       virtual void unregisterPlannerStartObserver(const std::shared_ptr<PlannerStartObserver<STATE_DIMENSION,CONTROL_DIMENSION>>& observer){
           this->planner_start_observers.erase(std::find(planner_start_observers.begin(),planner_start_observers.end(),observer));
       }

        /***
         * get the solution data for solution update observers.
         * @param forward_states
         * @param backward_states
         * @param connect_states
         * @param forward_control
         * @param backward_control
         * @param connect_control
         * @param forward_durations
         * @param backward_durations
         * @param connect_durations
         */
        void getSolutionVectors(std::vector<StateType> &forward_states,
                                std::vector<StateType> &backward_states,
                                std::vector<StateType> &connect_states,
                                std::vector<ControlType> &forward_control,
                                std::vector<ControlType> &backward_control,
                                std::vector<ControlType> &connect_control,
                                std::vector<double> &forward_durations,
                                std::vector<double> &backward_durations,
                                std::vector<double> &connect_durations,
                                std::string& solution_string) {
            forward_states.clear();
            backward_states.clear();
            connect_states.clear();
            forward_control.clear();
            backward_control.clear();
            connect_control.clear();
            forward_durations.clear();
            backward_durations.clear();
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
            backward_durations.push_back(0);
            backward_control.push_back(ControlType(_dynamic_system->getControlDimension()));
            while (node) {
                backward_states.push_back(node->getState());
                backward_control.push_back(node->getEdgeControl());
                backward_durations.push_back(node->getEdgeDuration());
                node = node->getParent();
            }
            backward_durations.pop_back();
            backward_control.pop_back();

            if (_planner_connection != nullptr && _planner_connection->_end_states == _best_goal) {
                connect_states = _planner_connection->_states;
                connect_control = _planner_connection->_controls;
                connect_durations = _planner_connection->_durations;
            }
            std::reverse(forward_states.begin(), forward_states.end());
            std::reverse(forward_control.begin(), forward_control.end());
            std::reverse(forward_durations.begin(), forward_durations.end());

            auto maxDistance = [this](const StateType & s1,const StateType& s2){
                double t1 = 0.0;
                for(auto i=0;i<STATE_DIMENSION/2;++i){
                    t1 = std::max( (s1.segment(2*i,2)-s2.segment(2*i,2)).norm(),t1);
                }
                return t1;
            };

            VariablesGrid forward_grid;
            auto current_state = forward_states.front();
            forward_grid.addVector(current_state*1e6,0.0);

            double t = 0.0;
            for(auto i=0;i<forward_states.size()-1;++i){
                auto temp_state = forward_states[i];
                //std::cout<<"Start State:"<<current_state.transpose()<<'\n';
                auto temp_control=forward_control[i+1];
                int steps = forward_durations[i+1]/_dynamic_system->getStepSize();
                for(auto j=0;j<steps;++j){
                    temp_state = forward(temp_state,temp_control,_dynamic_system->getStepSize());
                    t+=_dynamic_system->getStepSize();
                    if(maxDistance(temp_state,current_state)>SystemConfig::max_distance){
                        current_state = temp_state;
                        forward_grid.addVector(current_state*1e6, forward_grid.getLastTime() + t);
                        t = 0.0;
                    }
                }
            }

            if(!connect_states.empty()) {
                for (auto i = 0; i < connect_states.size() - 1; ++i) {
                    auto temp_state = connect_states[i];
                    auto temp_control = connect_control[i + 1];
                    int steps = connect_durations[i+1] / _dynamic_system->getStepSize();
                    for (auto j = 0; j < steps; ++j) {
                        temp_state = forward(temp_state, temp_control, _dynamic_system->getStepSize());
                        t+=_dynamic_system->getStepSize();
                        if(maxDistance(temp_state,current_state)>SystemConfig::max_distance){
                            current_state = temp_state;
                            forward_grid.addVector(current_state*1e6, forward_grid.getLastTime() + t);
                            t = 0.0;
                        }
                    }
                }
            }

            VariablesGrid backward_grid;
            backward_grid.addVector(backward_states.back(),0.0);
            for(int i=backward_states.size()-1;i>0;--i){
                auto current_state = backward_states[i];
                auto current_control=backward_control[i];
                int steps = backward_durations[i]/_dynamic_system->getStepSize();
                for(auto j=0;j<steps;++j){
                    current_state = forward(current_state,current_control,_dynamic_system->getStepSize());
                    backward_grid.addVector(current_state,backward_grid.getLastTime()+_dynamic_system->getStepSize());
                }
            }
            for(int i=backward_grid.getNumPoints()-1;i>=0;--i){
                t+=_dynamic_system->getStepSize();
                if(maxDistance(backward_grid.getVector(i),current_state)>25e-6){
                    current_state = backward_grid.getVector(i);
                    forward_grid.addVector(current_state*1e6, forward_grid.getLastTime() + t);
                    t = 0.0;
                }
            }

            if(t>_dynamic_system->getStepSize()){
                forward_grid.addVector(backward_grid.getVector(0)*1e6, forward_grid.getLastTime()+t);
            }

            std::stringstream ss;
            forward_grid.print(ss,"","","",10,6,"\t","\n");
            solution_string=ss.str();
        }

        /***
         * update best cost
         */
        virtual void updateBestCost() {
            if (_best_goal.first == nullptr || _best_goal.second == nullptr){
                _best_cost = std::numeric_limits<double>::max();
            }else {
                double cost = _best_goal.first->getCost() + _best_goal.second->getCost();
                if (_planner_connection != nullptr && _planner_connection->_end_states.first == _best_goal.first
                    && _planner_connection->_end_states.second == _best_goal.second) {
                    cost += _planner_connection->getTotalCost();
                }
                _best_cost = cost;
            }
        }

        double getBestCost(){
            return _best_cost;
        }

        /***
         * notify the planner to stop. here the run_flag can be set to false. Also can call a function to stop the process in dynamic system
         */
        virtual void stop() {
            _run_flag = false;
            _dynamic_system->stop();
        }

        /***
         * notify observers planning started
         * @param planner_name planner name
         * @param image_name image name
         * @param reference reference path
         */
       virtual void notifyPlannerStart(const std::string& planner_name,const std::string& image_name,const VariablesGrid& reference) {
           for(auto observer: planner_start_observers){
               observer->onPlannerStart(planner_name,
                                        _dynamic_system->getRobotCount(),
                                        _init_state,_target_state,
                                        reference,
                                        PlannerConfig::bidirection,
                                        PlannerConfig::optimization,
                                        PlannerConfig::total_time,
                                        PlannerConfig::goal_radius,
                                        PlannerConfig::integration_step,
                                        PlannerConfig::min_time_steps,
                                        PlannerConfig::max_time_steps,
                                        PlannerConfig::sst_delta_near,
                                        PlannerConfig::sst_delta_drain,
                                        PlannerConfig::optimization_distance,
                                        PlannerConfig::blossomM,
                                        PlannerConfig::blossomN,
                                        PlannerConfig::dominant_path_count,
                                        PlannerConfig::quality_decrease_factor,
                                        image_name);
           }
       }

        virtual void notifyNodeAdded(const StateType & state,TreeId id) {
           for(auto observer: node_added_observers){
               observer->onNodeAdded(state,id);
           }
        }

        /***
          * notify solution update observers. This function should be manually called when a solution is updated
          */
        virtual void notifySolutionUpdate() {
            std::vector<StateType> forward_state;
            std::vector<StateType> reverse_state;
            std::vector<StateType> connect_state;
            std::vector<ControlType> forward_control;
            std::vector<ControlType> reverse_control;
            std::vector<ControlType> connect_control;
            std::vector<double> forward_durations;
            std::vector<double> reverse_durations;
            std::vector<double> connect_durations;
            std::string solution_string;
            getSolutionVectors(forward_state,reverse_state,connect_state,
                               forward_control,reverse_control,connect_control,
                               forward_durations,reverse_durations,connect_durations,solution_string);

            if(!_run_flag)return;
            for(auto observer: solution_update_observers){
                observer->onSolutionUpdate(forward_state,reverse_state,connect_state,forward_control,reverse_control,connect_control,forward_durations,reverse_durations,connect_durations,solution_string);
            }
        }

    };
}


#endif //NANOWIREPLANNER_NANOWIRE_PLANNER_HPP
