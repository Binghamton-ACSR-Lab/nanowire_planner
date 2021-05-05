//
// Created by acsr on 4/26/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_SYSTEM_HPP
#define NANOWIREPLANNER_NANOWIRE_SYSTEM_HPP


#include <iostream>
#include <acado/acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include "nanowire_config.hpp"
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
        int _state_dimension;
        int _control_dimension;
        std::atomic_bool _run_flag;
        std::shared_ptr<NanowireConfig> _nanowire_config;
        std::shared_ptr<EpField> _field;

        Eigen::VectorXd _state_low_bound;
        Eigen::VectorXd _state_upper_bound;

        Eigen::VectorXd _control_low_bound;
        Eigen::VectorXd _control_upper_bound;

        Eigen::MatrixXd _mat_theta;

        const double _e0 = 8.85e-12;
        const double _em = 2.17 * _e0;
        const double _mu = 216.95e-3;
        const double _wire_radius = 10e-6;

        //std::shared_ptr<ReferencePath> reference_path;

    public:
        NanowireSystem() = delete;

        /***
     * constructor
     * @param nanowire_config a NanowireConfig pointer where store the config of the system
     */
        NanowireSystem(const std::shared_ptr<NanowireConfig>& nanowire_config):_nanowire_config(nanowire_config){
            _field = std::make_shared<EpField>(nanowire_config);
            _field->readFile();
            auto nanowire_count = _nanowire_config->getNanowireCount();
            _state_dimension = 2 * nanowire_count;

            //Eigen::VectorXd zp(2*nanowire_count);
            auto zp_vec = nanowire_config->getZetaPotentialVec();
            Eigen::Map<Eigen::VectorXd> zp(zp_vec.data(), 2 * nanowire_count);
            /*
        for(auto i=0;i<2*nanowire_count;++i)
            zp(i) = zp_vec[i];
        */
            setZetaPotential(zp);

            _state_low_bound.resize(2 * nanowire_count);
            _state_low_bound.setConstant(0);

            _state_upper_bound.resize(2 * nanowire_count);
            for (auto i = 0; i < nanowire_count; ++i) {
                _state_upper_bound(2 * i) =
                        nanowire_config->getColumnSpace() * (nanowire_config->getElectrodesCols() - 1);
                _state_upper_bound(2 * i + 1) =
                        nanowire_config->getRowSpace() * (nanowire_config->getElectrodesRows() - 1);
            }

            _control_dimension = nanowire_config->getElectrodesRows() * nanowire_config->getElectrodesCols();
            _control_low_bound.resize(_control_dimension);
            _control_low_bound.setZero();
            _control_upper_bound.resize(_control_dimension);
            _control_upper_bound.setOnes();
        }

        NanowireSystem(const NanowireSystem &) = delete;

        NanowireSystem operator=(const NanowireSystem &) = delete;

        ~NanowireSystem() = default;

        IVector getGrid(DVector state){
            state/=Config::sst_delta_drain;
            return state.cast<int>();
        }

        /***
         * set zeta potential
         * @param zeta_potential
         */
        void setStart(){
            _run_flag = true;
        }

        void stop(){
            _run_flag = false;
        }

        size_t getControlDimension() const{
            return _control_dimension;
        }

        void setControlDimension(size_t dimension){
            _control_dimension = dimension;
        }

        size_t getStateDimension() const{
            return _state_dimension;
        }

        void setStateDimension(size_t dimension){
            _state_dimension = dimension;
        }



        void setZetaPotential(const Eigen::VectorXd &zeta_potential) {
            assert(zeta_potential.size() == _state_dimension);
            _mat_theta = zeta_potential.asDiagonal();
        }

        auto getConfig(){
            return _nanowire_config;
        }

        /***
     * override DynamicSystem function
     * @return
     */
     /*
        double getWidth() const override {
            return nanowire_config->getColumnSpace() * (nanowire_config->getElectrodesCols() - 1) * 1e6;
        }
    */
        /***
     * override DynamicSystem function
     * @return
     */
     /*
        double getHeight() const override {
            return nanowire_config->getRowSpace() * (nanowire_config->getElectrodesRows() - 1) * 1e6;
        }
    */


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
            std::vector<double> dist(_nanowire_config->getNanowireCount());
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
            Eigen::VectorXd state(_state_dimension);
            for(auto i=0;i<_state_dimension;++i){
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
            for (auto i = 0; i < _control_dimension; ++i)
                control(i) = randomInteger(_control_low_bound(i),_control_upper_bound(i));
            return control;
        }

        /***
     * override DynamicSystem function
     * @return
     */
        bool
        forwardPropagateBySteps(const Eigen::VectorXd &init_state, const Eigen::VectorXd &control, double step_length,
                                int steps, Eigen::VectorXd &result_state, double &duration) {
            Eigen::MatrixXd mat_E;
            if (init_state.size() != _state_dimension)
                return false;
            result_state = init_state;
            auto nanowire_count = _nanowire_config->getNanowireCount();
            Eigen::VectorXd x(nanowire_count), y(nanowire_count);

            auto max_theta_pt = std::max_element(_mat_theta.data(), _mat_theta.data() + 2 * nanowire_count,
                                                 [](double &aa, double &bb) {
                                                     return (std::abs(aa) < std::abs(bb));
                                                 });
            double int_step = std::max(int(std::round(1.0 / std::abs(*max_theta_pt))), 1) * step_length;

            for (auto i = 0; i < steps; ++i) {
                for (int k = 0; k < nanowire_count; k++) {
                    x(k) = result_state(2 * k);
                    y(k) = result_state(2 * k + 1);
                }
                _field->getField(x, y, mat_E, nanowire_count);
                auto mat_velocity = _mat_theta * mat_E * control * _em / _mu;
                result_state += int_step * mat_velocity;
                //MAX_VELOCITY = MAX_VELOCITY > mat_velocity.norm()/wire_count ? MAX_VELOCITY : mat_velocity.norm()/wire_count;
                if (!validState(result_state))
                    return false;
            }
            duration = steps * int_step;
            return true;
        }


        /***
     * override DynamicSystem function
     * @return
     */
        bool
        reversePropagateBySteps(const Eigen::VectorXd &init_state, const Eigen::VectorXd &control, double step_length,
                                int step, Eigen::VectorXd &result_state, double &duration){
            return forwardPropagateBySteps(init_state, control, step_length, step, result_state, duration);
        }

        /***
     * override DynamicSystem function
     * @return
     */
        std::vector<NodePtr>
        getNearNodeByCount(const Eigen::VectorXd &state, const KdTreeType &tree, int count)  {
            if (state.size() != _state_dimension)
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
                                        double step_length, double max_distance, Eigen::VectorXd &result_state,
                                        double &duration) {
            Eigen::MatrixXd mat_E;
            auto nanowire_count = _nanowire_config->getNanowireCount();
            if (init_state.size() != nanowire_count)
                return false;
            result_state = init_state;
            Eigen::VectorXd x(nanowire_count), y(nanowire_count);
            auto data = _mat_theta.diagonal().data();
            auto max_theta_pt = std::max_element(_mat_theta.data(), _mat_theta.data() + 2 * nanowire_count,
                                                 [](double &aa, double &bb) {
                                                     return (std::abs(aa) < std::abs(bb));
                                                 });
            double int_step = std::max(int(std::round(1.0 / std::abs(*max_theta_pt))), 1) * step_length;
            int step = 0;
            duration = 0;
            while (distance(init_state, result_state) < max_distance) {
                for (int k = 0; k < nanowire_count; k++) {
                    x(k) = result_state(2 * k);
                    y(k) = result_state(2 * k + 1);
                }
                _field->getField(x, y, mat_E, nanowire_count);
                auto mat_velocity = _mat_theta * mat_E * control * _em / _mu;
                result_state += int_step * mat_velocity;
                if (!validState(result_state))
                    return false;
            }
            duration = step * int_step;
            return true;
        }

        /***
     * override DynamicSystem function
     * @return
     */
        bool reversePropagateByDistance(const Eigen::VectorXd &init_state, const Eigen::VectorXd &control,
                                        double step_length, double max_distance, Eigen::VectorXd &result_state,
                                        double &duration) {
            return forwardPropagateByDistance(init_state, control, step_length, max_distance, result_state, duration);
        }

        /***
     * override DynamicSystem function
     * @return
     */
        double getHeuristic(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2)  {
            ///1200e-6/100 is the max velocity;
            return maxDistance(state1, state2) / (1200e-6 / 100);
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
            return _nanowire_config->getNanowireCount();
        }

        /***
     * override DynamicSystem function
     * @return
     */
        bool connect(const Eigen::VectorXd &start, const Eigen::VectorXd &target, double int_step,
                     Eigen::MatrixXd &vec_state,
                     Eigen::MatrixXd &vec_control,
                     Eigen::VectorXd &vec_duration)  {
            auto nanowire_count = _nanowire_config->getNanowireCount();
            ACADO::DVector x0(start);
            ACADO::DVector xt(target);

            //std::cout<<x0<<std::endl<<xt<<std::endl;

            auto states = std::make_shared<ACADO::VariablesGrid>();//=new VariablesGrid();
            auto controls = std::make_shared<ACADO::VariablesGrid>();//=new VariablesGrid();

            states->addVector(x0, 0);


            auto max_theta_pt = std::max_element(_mat_theta.data(), _mat_theta.data() + 2 * nanowire_count,
                                                 [](double &aa, double &bb) {
                                                     return (std::abs(aa) < std::abs(bb));
                                                 });
            double int_step_max = std::max(int(1.0 / std::abs(*max_theta_pt)), 1) * int_step;


            if (!optimize(x0, xt, int_step_max, states, controls))
                return false;

            vec_state.resize(controls->getNumPoints(), 2 * nanowire_count);
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
        bool optimize(ACADO::DVector x0, ACADO::DVector xt, double int_step,
                      std::shared_ptr<ACADO::VariablesGrid> states,
                      std::shared_ptr<ACADO::VariablesGrid> controls) {
            USING_NAMESPACE_ACADO
            auto nanowire_count = _nanowire_config->getNanowireCount();

            ///define ocp parameters
            ACADO::DifferentialState x("", 2 * nanowire_count, 1);
            ACADO::Control u("", _control_dimension, 1);
            ACADO::Parameter T;

            ///horizontal steps
            double steps = 20;
            x.clearStaticCounters();
            u.clearStaticCounters();
            T.clearStaticCounters();

            Eigen::VectorXd x_position(nanowire_count), y_position(nanowire_count);
            int iterator = 0;

            while ((x0 - xt).norm() > Config::goal_radius && _run_flag) {
                if (iterator++ >= 5)
                    return false;

                for (auto i = 0; i < nanowire_count; ++i) {
                    x_position(i) = (x0(2 * i) + xt(2 * i)) / 2;//x0(2*i);
                    y_position(i) = (x0(2 * i + 1) + xt(2 * i + 1)) / 2;//x0(2*i+1);
                }

                Eigen::MatrixXd mat_E;
                _field->getField(x_position, y_position, mat_E, nanowire_count);
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
                for (int n = 0; n < 2 * nanowire_count; ++n) {
                    ocp.subjectTo(_state_low_bound(n) <= x(n) <= _state_upper_bound(n));
                }
                ///maximum time
                ocp.subjectTo(int_step <= T <= 500);

                ///define algorithm as solve
                ACADO::OptimizationAlgorithm algorithm(ocp);
                algorithm.set(ACADO::MAX_NUM_ITERATIONS, 15);
                algorithm.set(ACADO::KKT_TOLERANCE, Config::goal_radius);
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
                    if (!optimize(x0, xt_new, int_step, states, controls))
                        return false;
                    x0 = states->getLastVector();
                    x.clearStaticCounters();
                    u.clearStaticCounters();
                    T.clearStaticCounters();
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
                        for (int k = 0; k < nanowire_count; k++) {
                            x_position(k) = curr_state(2 * k);
                            y_position(k) = curr_state(2 * k + 1);
                        }
                        _field->getField(x_position, y_position, mat_E, nanowire_count);

                        ACADO::DVector mat_velocity = _mat_theta * mat_E * vec_u * _em / _mu;
                        curr_state += mat_velocity * int_step;
                        current_time += int_step;

                        if (!validState(curr_state))
                            return false;
                    }

                    auto compare_vector = vg_states.getVector(index + 1);

                    if ((curr_state - compare_vector).norm() > Config::goal_radius && index > 0)
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
            return true;
        }

        /*
        void setReferencePath(const std::shared_ptr<ReferencePath>& path){
            reference_path = path;
        }
        VariablesGrid getReferencePathState(){
            return reference_path->getStates();
        }*/




    protected:
        const double MAX_VELOCITY = 1e-7;
        std::vector<int> dominant_index;
        /***
     * override DynamicSystem function
     * @return
     */
        bool validState(const Eigen::VectorXd &state) {
            auto nanowire_count = _nanowire_config->getNanowireCount();
            for (unsigned i = 0; i < nanowire_count - 1; i++) {
                for (unsigned j = i + 1; j < nanowire_count; j++) {
                    if ((state.segment(2 * i, 2) - state.segment(2 * j, 2)).norm() < _wire_radius) {
                        return false;
                    }
                }
            }
            for (int i = 0; i < nanowire_count; i++) {
                if (state[2 * i] <= _state_low_bound(0) || state[2 * i] >= _state_upper_bound(0)
                    || state[2 * i + 1] <= _state_low_bound(1) || state[2 * i + 1] >= _state_upper_bound(1))
                    return false;
            }
            return true;
        }
    };


}

#endif //NANOWIREPLANNER_NANOWIRE_SYSTEM_HPP
