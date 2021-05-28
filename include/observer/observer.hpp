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
    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class SolutionUpdateObserver {
        using STATE_TYPE = Eigen::Matrix<double,STATE_DIMENSION,1>;
        using CONTROL_TYPE = Eigen::Matrix<double,CONTROL_DIMENSION,1>;
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
                const std::vector<STATE_TYPE> &forward_states,
                const std::vector<STATE_TYPE> &reverse_states,
                const std::vector<STATE_TYPE> &connect_states,
                const std::vector<CONTROL_TYPE> &forward_control,
                const std::vector<CONTROL_TYPE> &reverse_control,
                const std::vector<CONTROL_TYPE> &connect_control,
                const std::vector<double> &forward_durations,
                const std::vector<double> &reverse_durations,
                const std::vector<double> &connect_durations,const std::string& solution_string) = 0;

        virtual ~SolutionUpdateObserver() =default;
    };


    /***
     * planner start observer
     */
    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class PlannerStartObserver
    {
        using STATE_TYPE = Eigen::Matrix<double,STATE_DIMENSION,1>;
    public:
        PlannerStartObserver()=default;
        PlannerStartObserver(const PlannerStartObserver&) = delete;
        PlannerStartObserver& operator=(const PlannerStartObserver&) = delete;

        /***
         *
         * @param type
         * @param robot_count
         * @param init_state
         * @param target_state
         * @param reference_path
         * @param image_name
         */
        virtual void onPlannerStart(
                std::string type,
                int robot_count,
                const STATE_TYPE& init_state,
                const STATE_TYPE& target_state,
                const ACADO::VariablesGrid& reference_path,
                const std::string& image_name
        ) =0;
        virtual ~PlannerStartObserver()=default;
    };

    /***
     * node added observer
     */
    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class NodeAddedObserver
    {
        using STATE_TYPE = Eigen::Matrix<double,STATE_DIMENSION,1>;
        using CONTROL_TYPE = Eigen::Matrix<double,CONTROL_DIMENSION,1>;
    public:
        NodeAddedObserver()=default;
        NodeAddedObserver(const NodeAddedObserver&) = delete;
        NodeAddedObserver& operator=(const NodeAddedObserver&) = delete;


        virtual void onNodeAdded(const STATE_TYPE& state,TreeId id) = 0;
        virtual ~NodeAddedObserver()=default;
    };

    /***
     * an interface to show string message
     */
    class MessageDisplayer{
    public:
        MessageDisplayer() = default;
        virtual ~MessageDisplayer()=default;

        virtual void displayMessage(const std::string& msg)=0;
    };
}



#endif //NANOWIREPLANNER_OBSERVER_HPP
