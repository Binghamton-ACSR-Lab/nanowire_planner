//
// Created by acsr on 4/30/21.
//

#ifndef NANOWIREPLANNER_OBSERVER_HPP
#define NANOWIREPLANNER_OBSERVER_HPP
#include "nanowire_utility.hpp"
#include "acado/acado_toolkit.hpp"
namespace acsr {

    /***
     * solution updated observer
     */
    class SolutionUpdateObserver {
    public:
        SolutionUpdateObserver() = default;

        SolutionUpdateObserver(const SolutionUpdateObserver &) = delete;

        SolutionUpdateObserver &operator=(const SolutionUpdateObserver &) = delete;

        /***
         *
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
        virtual void onSolutionUpdate(
                const std::vector<Eigen::VectorXd> &forward_states,
                const std::vector<Eigen::VectorXd> &reverse_states,
                const std::vector<Eigen::VectorXd> &connect_states,
                const std::vector<Eigen::VectorXd> &forward_control,
                const std::vector<Eigen::VectorXd> &reverse_control,
                const std::vector<Eigen::VectorXd> &connect_control,
                const std::vector<double> &forward_durations,
                const std::vector<double> &reverse_durations,
                const std::vector<double> &connect_durations,const std::string& solution_string) = 0;

        virtual ~SolutionUpdateObserver() =default;
    };


    /***
     * planner start observer
     */
    class PlannerStartObserver
    {
    public:
        PlannerStartObserver()=default;
        PlannerStartObserver(const PlannerStartObserver&) = delete;
        PlannerStartObserver& operator=(const PlannerStartObserver&) = delete;

        /***
         * @brief notify obserser when planner start
         * @param type
         * @param robot_count
         * @param init_state
         * @param target_state
         * @param bidrectional
         * @param optimization
         * @param stop_type
         * @param stop_value
         * @param goal_radius
         * @param step_size
         * @param min_steps
         * @param max_steps
         * @param sst_delta_near
         * @param sst_delta_drain
         * @param optimization_distance
         * @param m
         * @param n
         * @param a
         * @param b
         */
        virtual void onPlannerStart(
                std::string type,
                int robot_count,
                const Eigen::VectorXd& init_state,
                const Eigen::VectorXd& target_state,
                const ACADO::VariablesGrid& reference_path,
                bool bidirectional,
                bool optimization,
                double stop_value,
                double goal_radius,
                double step_size,
                int min_steps,int max_steps, double sst_delta_near, double sst_delta_drain,
                double optimization_distance,
                int m,int n,int dominant_path_count,double quality_decrease_factor,
                const std::string& image_name
        ) =0;

        virtual ~PlannerStartObserver()=default;
    };

    /***
     * node added observer
     */
    class NodeAddedObserver
    {
    public:
        NodeAddedObserver()=default;
        NodeAddedObserver(const NodeAddedObserver&) = delete;
        NodeAddedObserver& operator=(const NodeAddedObserver&) = delete;


        virtual void onNodeAdded(const Eigen::VectorXd& state,TreeId id) = 0;

        virtual ~NodeAddedObserver()=default;
    };

    class MessageDisplayer{
    public:
        MessageDisplayer() = default;
        virtual ~MessageDisplayer()=default;

        virtual void displayMessage(const std::string& msg)=0;
    };
}



#endif //NANOWIREPLANNER_OBSERVER_HPP
