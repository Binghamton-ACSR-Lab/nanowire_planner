//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_SYSTEM_HPP
#define NANOWIREPLANNER_NANOWIRE_SYSTEM_HPP


#include <iostream>
#include <acado/acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include "config/nanowire_config.hpp"
#include "config/planner_config.hpp"
#include "tree_node.hpp"
#include "nanowire_utility.hpp"
#include "svg.hpp"
namespace acsr {

    USING_NAMESPACE_ACADO
    template <int NANOWIRE_COUNT>
    class ReferencePath
    {
    public:
        ReferencePath()=default;
        virtual ~ReferencePath()=default;

        /***
         * read reference trajectory file
         * @param filename file name
         * @return
         */
        bool readFile(std::string filename){
            if(!boost::filesystem::exists(filename)){
                std::cout<<filename<<" does not exist\n";
                return false;
            }
            _path.read(filename.c_str());
            //_path.print();
            return true;
        }

        /***
         * get path
         * @return path
         */
        const VariablesGrid &getStates() const{
            return _path;
        }

        /***
         * get state at time
         * @param time time
         * @return state
         */
        DVector getState(double time){
            return _path.linearInterpolation(time);
        }

        /***
         * get total cost time
         * @return time
         */
        double getMaxTime(){
            return _path.getLastTime();
        }

    private:
        VariablesGrid _path;
    };

    template <int NANOWIRE_COUNT,int ELECTRODE_COUNT>
    class NanowireSystem  {
        using StateType = Eigen::Matrix<double,2*NANOWIRE_COUNT,1>;
        using ControlType = Eigen::Matrix<double,ELECTRODE_COUNT,1>;
    private:

        std::atomic_bool _run_flag;
        std::shared_ptr<EpField> _field;

        StateType _state_low_bound;
        StateType _state_upper_bound;
        ControlType _control_low_bound;
        ControlType _control_upper_bound;

        Eigen::Matrix<double,NANOWIRE_COUNT,1> _current_height;
        Eigen::Matrix<double,2*NANOWIRE_COUNT,2*NANOWIRE_COUNT> _mat_theta;

        const double _e0 = 8.85e-12;
        const double _em = 2.17 * _e0;
        const double _mu = 216.95e-3;
        const double _wire_radius = 5e-6;

        double _step_length{};
        int _field_dimension{};

    public:
        NanowireSystem() = delete;
        NanowireSystem(const NanowireSystem &) = delete;
        NanowireSystem operator=(const NanowireSystem &) = delete;

        /***
         * constructor
         * @param wire_count nanowire count
         * @param field_dimension electrical field dimension
         */
        NanowireSystem(int field_dimension) : _field_dimension(field_dimension){
            _field = std::make_shared<EpField>(_field_dimension);
            _field->readFile();
            //_n_wire = wire_count;

            _state_low_bound.resize(2 * NANOWIRE_COUNT);
            _state_low_bound.setConstant(10e-6);

            _state_upper_bound.resize(2 * NANOWIRE_COUNT);
            for (auto i = 0; i < NANOWIRE_COUNT; ++i) {
                _state_upper_bound(2 * i) =
                        NanowireConfig::electrodes_space * (NanowireConfig::electrodes_columns - 1)-10e-6;
                _state_upper_bound(2 * i + 1) =
                        NanowireConfig::electrodes_space * (NanowireConfig::electrodes_rows - 1)-10e-6;
            }

            //_control_dimension = NanowireConfig::electrodes_rows * NanowireConfig::electrodes_columns;
            _control_low_bound.resize(ELECTRODE_COUNT);
            _control_low_bound.setZero();
            _control_upper_bound.resize(ELECTRODE_COUNT);
            _control_upper_bound.setOnes();
        }

        virtual ~NanowireSystem() = default;

        /***
         * initialize
         * @param zeta zeta potential
         * @param height nanowire height, ignored if dimension = 2
         */
        void init(const Eigen::Matrix<double,2*NANOWIRE_COUNT,1>& zeta, const Eigen::Matrix<double,NANOWIRE_COUNT,1>& height){
            setZetaPotential(zeta);
            setHeight(height);
            updateStepLength();
        }

        /***
         * round state
         * @param state
         * @return
         */
        Eigen::Matrix<int,2*NANOWIRE_COUNT,1> getGrid(Eigen::Matrix<double,2*NANOWIRE_COUNT,1> state){
            state/=PlannerConfig::sst_delta_drain;
            return state.template cast<int>();
        }

        /***
         * get step size
         * @return
         */
        double getStepSize(){
            return _step_length;
        }

        /***
         * reset run flag
         */
        void reset(){
            _run_flag = true;
        }

        /***
         * stop, terminate thread
         */
        void stop(){
            _run_flag = false;
        }

        /***
         * distance of two state
         * @param state1
         * @param state2
         * @return distance
         */
        double distance(const Eigen::Matrix<double,2*NANOWIRE_COUNT,1> &state1, const Eigen::Matrix<double,2*NANOWIRE_COUNT,1> &state2) {
            return (state1 - state2).norm();
        }

        /***
         * get the maximum distance for each nanowire between two states
         * @param state1
         * @param state2
         * @return max distance
         */
        double maxDistance(const StateType &state1, const StateType &state2) const {
            std::vector<double> dist(NANOWIRE_COUNT);
            for (auto i = 0; i < dist.size(); ++i) {
                dist[i] = (state1.segment(2 * i, 2) - state2.segment(2 * i, 2)).norm();
            }
            return *std::max_element(dist.begin(), dist.end());
        }

        /***
         * generate a random state within boundary
         * @return state
         */
        StateType randomState() {
            StateType state;
            for(auto i=0;i<2*NANOWIRE_COUNT;++i){
                state(i) = randomDouble(_state_low_bound(i),_state_upper_bound(i));
            }
            //std::cout<<state<<std::endl;
            return state;

        }

        /***
         * generate a random control input within boundary
         * @return random control
         */
        ControlType randomControl() {
            ControlType control;
            if(PlannerConfig::intermedia_control) {
                for (auto i = 0; i < ELECTRODE_COUNT; ++i)
                    control(i) = randomDouble(_control_low_bound(i),_control_upper_bound(i));
                double p = 1.0 / control.maxCoeff();
                return p*control;
            }else{
                for (auto i = 0; i < ELECTRODE_COUNT; ++i)
                    control(i) = randomInteger(_control_low_bound(i),_control_upper_bound(i));
                return control;
            }
        }

        /***
         * forward propagate
         * @param init_state init state
         * @param control input control
         * @param steps steps
         * @param result_state final state
         * @param duration total cost
         * @return true if successfully propagated
         */
        bool
        forwardPropagateBySteps(const StateType &init_state, const ControlType &control,
                                int steps, StateType &result_state, double &duration) {
            Eigen::MatrixXd mat_E;
            auto temp_mat_theta = _mat_theta * _em / _mu;
            result_state = init_state;
            for (auto i = 0; i < steps; ++i) {
                _field->getField<NANOWIRE_COUNT>(result_state,_current_height, mat_E);
                auto mat_velocity = temp_mat_theta * mat_E * control;
                result_state += _step_length * mat_velocity;
                if (!validState(result_state))
                    return false;
            }
            duration = steps * _step_length;
            return true;
        }

        /***
         * backward propaget
         * @param init_state inti state
         * @param control input control
         * @param step steps
         * @param result_state final state
         * @param duration total cost
         * @return true if successfully propagated
         */
        bool
        backwardPropagateBySteps(const StateType &init_state, const ControlType &control, //double step_length,
                                int step, StateType &result_state, double &duration){
            return forwardPropagateBySteps(init_state, control, step, result_state, duration);
        }



        bool forwardPropagateByDistance(const StateType &init_state, const ControlType &control,
                                        double max_distance, StateType &result_state,
                                        double &duration) {
            Eigen::MatrixXd mat_E;
            //auto nanowire_count = _nanowire_config->getNanowireCount();
            if (init_state.size() != NANOWIRE_COUNT)
                return false;
            result_state = init_state;

            int step = 0;
            duration = 0;
            auto temp_mat_theta = _mat_theta * _em / _mu;
            while (distance(init_state, result_state) < max_distance) {
                _field->getField(result_state,_current_height, mat_E, NANOWIRE_COUNT);
                auto mat_velocity = temp_mat_theta * mat_E * control;
                result_state += _step_length * mat_velocity;
                if (!validState(result_state))
                    return false;
            }
            duration = step * _step_length;
            return true;
        }

        /***
     * override DynamicSystem function
     * @return
     */
        bool backwardPropagateByDistance(const StateType &init_state, const StateType &control,
                                        double max_distance, ControlType &result_state,
                                        double &duration) {
            return forwardPropagateByDistance(init_state, control,max_distance, result_state, duration);
        }

        /***
     * override DynamicSystem function
     * @return
     */
        double getHeuristic(const StateType &state1, const StateType &state2)  {
            ///1200e-6/100 is the max velocity;
            return maxDistance(state1, state2) /1e-5;
        }

        /***
         * override DynamicSystem function
         * @return always false since this system can't be redirect
         */
        bool redirect(const StateType &start,
                      const StateType &target,
                      ControlType &controls,
                      double &duration) {
            std::cout << "Nanowire System Can't Be Redirected!/n";
            return false;
        }

        /***
         * override. Current always return empty vector. This function can be implemented.
         * @param start
         * @param target
         * @return
         */
        ControlType getGuideControl(const StateType &start,
                                        const StateType &target)  {
            return ControlType();
        }

        /***
         * override DynamicSystem function
         * @return
         */
        bool connect(const StateType &start, const StateType &target,
                     Eigen::MatrixXd &vec_state,
                     Eigen::MatrixXd &vec_control,
                     Eigen::VectorXd &vec_duration)  {

            ACADO::DVector x0(start);
            ACADO::DVector xt(target);


            auto states = std::make_shared<ACADO::VariablesGrid>();//=new VariablesGrid();
            auto controls = std::make_shared<ACADO::VariablesGrid>();//=new VariablesGrid();

            states->addVector(x0, 0);



            if (!optimize(x0, xt, states, controls)) {
                return false;
            }

            vec_state.resize(controls->getNumPoints(), 2 * NANOWIRE_COUNT);
            vec_control.resize(controls->getNumPoints(), ELECTRODE_COUNT);
            vec_duration.resize(controls->getNumPoints());
            for (ACADO::uint i = 0; i < states->getNumPoints() - 1; ++i) {
                vec_state.row(i) = states->getVector(i + 1);
                vec_control.row(i) = controls->getVector(i);
                vec_duration(i) = (states->getTime(i + 1) - states->getTime(i));
            }

            return true;
        }

        /***
         * override DynamicSystem function
         * @return
         */
        bool optimize(Eigen::Matrix<double,2*NANOWIRE_COUNT,1> x0, Eigen::Matrix<double,2*NANOWIRE_COUNT,1> xt,
                      std::shared_ptr<ACADO::VariablesGrid> states,
                      std::shared_ptr<ACADO::VariablesGrid> controls) {
            USING_NAMESPACE_ACADO
            //auto nanowire_count = _nanowire_config->getNanowireCount();

            ///define ocp parameters
            ACADO::DifferentialState x("", 2 * NANOWIRE_COUNT, 1);
            ACADO::Control u("", ELECTRODE_COUNT, 1);
            ACADO::Parameter T;

            ///horizontal steps
            double steps = 20;
            x.clearStaticCounters();
            u.clearStaticCounters();
            T.clearStaticCounters();

            //Eigen::VectorXd x_position(nanowire_count), y_position(nanowire_count);
            int iterator = 0;

            while ((x0 - xt).norm() > PlannerConfig::goal_radius && _run_flag) {
                if (iterator++ >= 5)
                    return false;

                auto state = (x0+xt)/2;

                Eigen::MatrixXd mat_E;
                _field->getField<NANOWIRE_COUNT>(state, _current_height, mat_E);
                ACADO::DMatrix B = _mat_theta * mat_E * _em / _mu;

                ///set differerntial equation
                ACADO::DifferentialEquation f(0.0, T);
                f << dot(x) == B * u;

                ///define ocp
                ACADO::OCP ocp(0, T, steps);
                ocp.minimizeMayerTerm(T);
                ocp.subjectTo(f);
                ocp.subjectTo(ACADO::AT_START, x == x0);
                ocp.subjectTo(ACADO::AT_END, x == xt);
                ///control constraints
                for (int k = 0; k < ELECTRODE_COUNT; ++k) {
                    ocp.subjectTo(0 <= u(k) <= 1);
                }
                ///state constraints
                for (int n = 0; n < 2 * NANOWIRE_COUNT; ++n) {
                    ocp.subjectTo(_state_low_bound(n) <= x(n) <= _state_upper_bound(n));
                }
                ///maximum time
                ocp.subjectTo(_step_length <= T <= 1000*_step_length);

                ///define algorithm as solve
                ACADO::OptimizationAlgorithm algorithm(ocp);
                algorithm.set(ACADO::MAX_NUM_ITERATIONS, 15);
                algorithm.set(ACADO::KKT_TOLERANCE, PlannerConfig::goal_radius);
                auto ret = algorithm.solve();

                ACADO::VariablesGrid vg_states, vg_controls;
                algorithm.getDifferentialStates(vg_states);
                algorithm.getControls(vg_controls);
                double current_cost = algorithm.getObjectiveValue();

                ///if the optimize process fails, we start another optimize process, with its target at the center of x0 and xt

                if (ret != SUCCESSFUL_RETURN) {
                    //return false;
                    //if(current_cost-int_step<0.5*int_step) {
                    auto xt_new = vg_states.getFirstVector();

                    //auto xt_new=(x0+xt)/2;
                    x.clearStaticCounters();
                    u.clearStaticCounters();
                    T.clearStaticCounters();
                    if (!optimize(x0, xt_new, states, controls))
                        return false;
                    x0 = states->getLastVector();
                    x.clearStaticCounters();
                    u.clearStaticCounters();
                    T.clearStaticCounters();
                    if(!_run_flag)return false;
                    continue;
                }


                double current_time = 0;
                double time_point = states->getLastTime();

                double time_interval = algorithm.getObjectiveValue() / steps;

                for (auto index = 0; index < steps; ++index) {
                    ACADO::DVector curr_state(states->getLastVector());
                    Eigen::MatrixXd mat_E;
                    auto vec_u = vg_controls.getVector(index);

                    controls->addVector(vec_u, current_time + time_point);
                    auto temp_mat_theta = _mat_theta * _em / _mu;
                    while (current_time < (index + 1) * time_interval) {
                        _field->getField<NANOWIRE_COUNT>(curr_state, _current_height, mat_E);
                        ACADO::DVector mat_velocity = temp_mat_theta * mat_E * vec_u;
                        curr_state += mat_velocity * _step_length;
                        current_time += _step_length;
                        if(!_run_flag || !validState(curr_state))return false;
                    }

                    auto compare_vector = vg_states.getVector(index + 1);

                    if ((curr_state - compare_vector).norm() > PlannerConfig::goal_radius && index > 0)
                        break;
                    states->addVector(curr_state, current_time + time_point);
                }
                x0 = states->getLastVector();
                x.clearStaticCounters();
                u.clearStaticCounters();
                T.clearStaticCounters();
            }
            x.clearStaticCounters();
            u.clearStaticCounters();
            T.clearStaticCounters();
            if(!_run_flag)return false;
            return true;
        }

    protected:
        const double MAX_VELOCITY = 1e-7;

        /***
         * override DynamicSystem function
         * @return
         */
        bool validState(const StateType &state) {
            //auto nanowire_count = _nanowire_config->getNanowireCount();
            for (unsigned i = 0; i < NANOWIRE_COUNT - 1; i++) {
                for (unsigned j = i + 1; j < NANOWIRE_COUNT; j++) {
                    if ((state.segment(2 * i, 2) - state.segment(2 * j, 2)).norm() < _wire_radius) {
                        return false;
                    }
                }
            }
            for (int i = 0; i < NANOWIRE_COUNT; i++) {
                if (state[2 * i] <= _state_low_bound(0) || state[2 * i] >= _state_upper_bound(0)
                    || state[2 * i + 1] <= _state_low_bound(1) || state[2 * i + 1] >= _state_upper_bound(1))
                    return false;
            }
            return true;
        }

        void updateStepLength(){
            auto max_theta_pt = std::max_element(_mat_theta.data(), _mat_theta.data() + 2 * NANOWIRE_COUNT,
                                                 [](double &aa, double &bb) {
                                                     return (std::abs(aa) < std::abs(bb));
                                                 });
            if(NanowireConfig::type=="cc60")
                _step_length = std::max((0.1 / std::abs(*max_theta_pt)), 0.1) * PlannerConfig::integration_step;
            else if(NanowireConfig::type=="cc600")
                _step_length = std::max(int(1.0 / std::abs(*max_theta_pt)), 1) * PlannerConfig::integration_step;
            if(_field_dimension==3) {
                auto x = _current_height.maxCoeff();
                _step_length *= 10 * 2e-7 / (-7e-7 * std::log(x/1.02) - 5e-6);
            }
            _step_length = std::round(_step_length*10)/10.0;
        }

        void setZetaPotential(const StateType &zeta_potential) {
            //assert(zeta_potential.size() == 2*NANOWIRE_COUNT);
            _mat_theta = zeta_potential.asDiagonal();
        }

        void setHeight(const Eigen::Matrix<double,NANOWIRE_COUNT,1>& height){
            _current_height = height;
        }
    };
}

#endif //NANOWIREPLANNER_NANOWIRE_SYSTEM_HPP
