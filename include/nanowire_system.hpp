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
    class ReferencePath
    {
    public:
        ReferencePath()=default;
        virtual ~ReferencePath()=default;

        bool readFile(std::string filename){
            if(!boost::filesystem::exists(filename)){
                std::cout<<filename<<" does not exist\n";
                return false;
            }
            _path.read(filename.c_str());
            _path.print();
            return true;
        }

        const VariablesGrid &getStates() const{
            return _path;
        }

        Eigen::MatrixXd getState(double time){
            return _path.linearInterpolation(time);
        }
        double getMaxTime(){
            return _path.getLastTime();
        }

    private:
        VariablesGrid _path;
    };

    class NanowireSystem  {
    private:
        //int _state_dimension;
        int _n_wire;
        int _control_dimension = 16;
        std::atomic_bool _run_flag;
        std::shared_ptr<EpField> _field;

        Eigen::VectorXd _state_low_bound;
        Eigen::VectorXd _state_upper_bound;

        Eigen::VectorXd _control_low_bound;
        Eigen::VectorXd _control_upper_bound;

        Eigen::VectorXd _current_height;

        Eigen::MatrixXd _mat_theta;

        const double _e0 = 8.85e-12;
        const double _em = 2.17 * _e0;
        const double _mu = 216.95e-3;
        const double _wire_radius = 5e-6;

        double _step_length = 0.1;
        int _field_dimension = 2;

        //std::shared_ptr<ReferencePath> reference_path;

    public:
        NanowireSystem() = delete;

        /***
     * constructor
     * @param nanowire_config a NanowireConfig pointer where store the config of the system
     */
        NanowireSystem(int wire_count, int field_dimension) : _field_dimension(field_dimension){
            _field = std::make_shared<EpField>(_field_dimension);
            _field->readFile();
            _n_wire = wire_count;

            _state_low_bound.resize(2 * _n_wire);
            _state_low_bound.setConstant(10e-6);

            _state_upper_bound.resize(2 * _n_wire);
            for (auto i = 0; i < _n_wire; ++i) {
                _state_upper_bound(2 * i) =
                        NanowireConfig::electrodes_space * (NanowireConfig::electrodes_columns - 1)-10e-6;
                _state_upper_bound(2 * i + 1) =
                        NanowireConfig::electrodes_space * (NanowireConfig::electrodes_rows - 1)-10e-6;
            }

            _control_dimension = NanowireConfig::electrodes_rows * NanowireConfig::electrodes_columns;
            _control_low_bound.resize(_control_dimension);
            _control_low_bound.setZero();
            _control_upper_bound.resize(_control_dimension);
            _control_upper_bound.setOnes();

        }


        NanowireSystem(const NanowireSystem &) = delete;

        NanowireSystem operator=(const NanowireSystem &) = delete;

        ~NanowireSystem() = default;

        void setParams(int field_dimension,const Eigen::VectorXd& zeta, const Eigen::VectorXd& height){
            _field_dimension = field_dimension;
            setZetaPotential(zeta);
            setHeight(height);
            updateStepLength();
        }

        IVector getGrid(DVector state){
            state/=PlannerConfig::sst_delta_drain;
            return state.cast<int>();
        }


        double getStepSize(){
            return _step_length;
        }

        /***
         * set zeta potential
         * @param zeta_potential
         */
        void reset(){
            _run_flag = true;
        }

        void stop(){
            _run_flag = false;
        }

        size_t getControlDimension() const{
            return _control_dimension;
        }

        /*
        void setControlDimension(size_t dimension){
            _control_dimension = dimension;
        }*/

        size_t getStateDimension() const{
            return 2*_n_wire;
        }



        /***
     * override DynamicSystem function
     * This function can be discuss further. Currently an euclidean distance is used. The maximum distance may be applied
     * @return
     */
        double distance(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2) {
            return (state1 - state2).norm();
        }

        /***
     * get the maximum distance for each nanowire between two states
     * @param state1
     * @param state2
     * @return
     */
        double maxDistance(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2) {
            std::vector<double> dist(_n_wire);
            for (auto i = 0; i < dist.size(); ++i) {
                dist[i] = (state1.segment(2 * i, 2) - state2.segment(2 * i, 2)).norm();
            }
            return *std::max_element(dist.begin(), dist.end());
        }

        /***
     * override DynamicSystem function
     * @return
     */
        Eigen::VectorXd randomState() {
            Eigen::VectorXd state(2*_n_wire);
            for(auto i=0;i<2*_n_wire;++i){
                state(i) = randomDouble(_state_low_bound(i),_state_upper_bound(i));
            }
            //std::cout<<state<<std::endl;
            return state;

        }

        /***
     * override DynamicSystem function
     * @return
     */
        Eigen::VectorXd randomControl() {
            Eigen::VectorXd control(_control_dimension);
            if(PlannerConfig::intermedia_control) {
                for (auto i = 0; i < _control_dimension; ++i)
                    control(i) = randomDouble(_control_low_bound(i),_control_upper_bound(i));
                double p = 1.0 / control.maxCoeff();
                return p*control;
            }else{
                for (auto i = 0; i < _control_dimension; ++i)
                    control(i) = randomInteger(_control_low_bound(i),_control_upper_bound(i));
                return control;
            }
            //return control;
        }

        /***
     * override DynamicSystem function
     * @return
     */
        bool
        forwardPropagateBySteps(const Eigen::VectorXd &init_state, const Eigen::VectorXd &control, //double step_length,
                                int steps, Eigen::VectorXd &result_state, double &duration) {
            Eigen::MatrixXd mat_E;
            if (init_state.size() != 2*_n_wire)
                return false;
            result_state = init_state;
            //auto nanowire_count = _nanowire_config->getNanowireCount();

            for (auto i = 0; i < steps; ++i) {
                _field->getField(result_state,_current_height, mat_E, _n_wire);
                auto mat_velocity = _mat_theta * mat_E * control * _em / _mu;
                result_state += _step_length * mat_velocity;
                if (!validState(result_state))
                    return false;
            }
            duration = steps * _step_length;
            return true;
        }


        /***
     * override DynamicSystem function
     * @return
     */
        bool
        reversePropagateBySteps(const Eigen::VectorXd &init_state, const Eigen::VectorXd &control, //double step_length,
                                int step, Eigen::VectorXd &result_state, double &duration){
            return forwardPropagateBySteps(init_state, control, step, result_state, duration);
        }

        /***
     * override DynamicSystem function
     * @return
     */
        std::vector<NodePtr>
        getNearNodeByCount(const Eigen::VectorXd &state, const KdTreeType &tree, int count)  {
            if (state.size() != 2*_n_wire)
                return std::vector<NodePtr> ();
            auto it = spatial::quadrance_neighbor_begin(tree, state);
            auto max_count = std::min(int(tree.size()), count);
            std::vector<NodePtr > near_nodes(max_count);
            for (auto i = 0; i < max_count; ++i) {
                near_nodes[i] = it->second;
                ++it;
            }
            return near_nodes;
        }

        /***
     * override DynamicSystem function
     * @return
     */
        bool forwardPropagateByDistance(const Eigen::VectorXd &init_state, const Eigen::VectorXd &control,
                                        double max_distance, Eigen::VectorXd &result_state,
                                        double &duration) {
            Eigen::MatrixXd mat_E;
            //auto nanowire_count = _nanowire_config->getNanowireCount();
            if (init_state.size() != _n_wire)
                return false;
            result_state = init_state;

            int step = 0;
            duration = 0;
            while (distance(init_state, result_state) < max_distance) {
                _field->getField(result_state,_current_height, mat_E, _n_wire);
                auto mat_velocity = _mat_theta * mat_E * control * _em / _mu;
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
        bool reversePropagateByDistance(const Eigen::VectorXd &init_state, const Eigen::VectorXd &control,
                                        double max_distance, Eigen::VectorXd &result_state,
                                        double &duration) {
            return forwardPropagateByDistance(init_state, control,max_distance, result_state, duration);
        }

        /***
     * override DynamicSystem function
     * @return
     */
        double getHeuristic(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2)  {
            ///1200e-6/100 is the max velocity;
            return maxDistance(state1, state2) /1e-5;
        }

        /***
     * override DynamicSystem function
     * @return
     */
        virtual std::pair<std::vector<NodePtr>,NodePtr>
        getNearNodeByRadiusAndNearest(const Eigen::VectorXd &state, const KdTreeType  &tree,
                                      double radius)  {
            auto it = spatial::euclidian_neighbor_begin(tree, state);
            auto max_count = std::min(int(tree.size()), 20);
            std::vector<NodePtr > near_nodes;
            auto nearest_node = it->second;
            for (auto i = 0; i < max_count; ++i) {
                if (distance(state, it->second->getState()) < radius) {
                    near_nodes.push_back(it->second);
                    ++it;
                } else {
                    //break;
                }
            }
            return {near_nodes, nearest_node};
        }

        /***
     * override DynamicSystem function
     * @return always false since this system can't be redirect
     */
        bool redirect(const Eigen::VectorXd &start,
                      const Eigen::VectorXd &target,
                      Eigen::VectorXd &controls,
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
        Eigen::VectorXd getGuideControl(const Eigen::VectorXd &start,
                                        const Eigen::VectorXd &target)  {
            return Eigen::VectorXd();
        }

        /***
     * override DynamicSystem function
     * @return
     */

        int getRobotCount() {
            return _n_wire;
        }

        /***
     * override DynamicSystem function
     * @return
     */
        bool connect(const Eigen::VectorXd &start, const Eigen::VectorXd &target,
                     Eigen::MatrixXd &vec_state,
                     Eigen::MatrixXd &vec_control,
                     Eigen::VectorXd &vec_duration)  {
            //auto nanowire_count = _nanowire_config->getNanowireCount();
            ACADO::DVector x0(start);
            ACADO::DVector xt(target);

            //std::cout<<x0<<std::endl<<xt<<std::endl;

            auto states = std::make_shared<ACADO::VariablesGrid>();//=new VariablesGrid();
            auto controls = std::make_shared<ACADO::VariablesGrid>();//=new VariablesGrid();

            states->addVector(x0, 0);


            auto max_theta_pt = std::max_element(_mat_theta.data(), _mat_theta.data() + 2 * _n_wire,
                                                 [](double &aa, double &bb) {
                                                     return (std::abs(aa) < std::abs(bb));
                                                 });


            if (!optimize(x0, xt, states, controls)) {
                return false;
            }

            vec_state.resize(controls->getNumPoints(), 2 * _n_wire);
            vec_control.resize(controls->getNumPoints(), _control_dimension);
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
        bool optimize(ACADO::DVector x0, ACADO::DVector xt,
                      std::shared_ptr<ACADO::VariablesGrid> states,
                      std::shared_ptr<ACADO::VariablesGrid> controls) {
            USING_NAMESPACE_ACADO
            //auto nanowire_count = _nanowire_config->getNanowireCount();

            ///define ocp parameters
            ACADO::DifferentialState x("", 2 * _n_wire, 1);
            ACADO::Control u("", _control_dimension, 1);
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
                _field->getField(state, _current_height, mat_E, _n_wire);
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
                for (int k = 0; k < _control_dimension; ++k) {
                    ocp.subjectTo(0 <= u(k) <= 1);
                }
                ///state constraints
                for (int n = 0; n < 2 * _n_wire; ++n) {
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

                ///explore with nonlinear mat_E using the control from the optimize process
                for (auto index = 0; index < steps; ++index) {
                    ACADO::DVector curr_state(states->getLastVector());
                    Eigen::MatrixXd mat_E;
                    auto vec_u = vg_controls.getVector(index);

                    controls->addVector(vec_u, current_time + time_point);
                    while (current_time < (index + 1) * time_interval) {
                        /*
                        for (int k = 0; k < nanowire_count; k++) {
                            x_position(k) = curr_state(2 * k);
                            y_position(k) = curr_state(2 * k + 1);
                        }*/

                        _field->getField(curr_state, _current_height, mat_E, _n_wire);

                        ACADO::DVector mat_velocity = _mat_theta * mat_E * vec_u * _em / _mu;
                        curr_state += mat_velocity * _step_length;
                        current_time += _step_length;
                        if(!_run_flag)return false;
                        if (!validState(curr_state))
                            return false;
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
        std::vector<int> dominant_index;

        /***
         * override DynamicSystem function
         * @return
         */
        bool validState(const Eigen::VectorXd &state) {
            //auto nanowire_count = _nanowire_config->getNanowireCount();
            for (unsigned i = 0; i < _n_wire - 1; i++) {
                for (unsigned j = i + 1; j < _n_wire; j++) {
                    if ((state.segment(2 * i, 2) - state.segment(2 * j, 2)).norm() < _wire_radius) {
                        return false;
                    }
                }
            }
            for (int i = 0; i < _n_wire; i++) {
                if (state[2 * i] <= _state_low_bound(0) || state[2 * i] >= _state_upper_bound(0)
                    || state[2 * i + 1] <= _state_low_bound(1) || state[2 * i + 1] >= _state_upper_bound(1))
                    return false;
            }
            return true;
        }

        void updateStepLength(){
            auto max_theta_pt = std::max_element(_mat_theta.data(), _mat_theta.data() + 2 * _n_wire,
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

        void setZetaPotential(const Eigen::VectorXd &zeta_potential) {
            assert(zeta_potential.size() == 2*_n_wire);
            _mat_theta = zeta_potential.asDiagonal();
        }

        void setHeight(const Eigen::VectorXd& height){
            _current_height = height;
        }
    };


}

#endif //NANOWIREPLANNER_NANOWIRE_SYSTEM_HPP
