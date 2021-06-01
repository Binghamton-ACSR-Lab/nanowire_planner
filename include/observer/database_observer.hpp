//
// Created by acsr on 6/1/21.
//
#ifndef NANOWIREPLANNER_DATABASE_OBSERVER_HPP
#define NANOWIREPLANNER_DATABASE_OBSERVER_HPP

#include <SQLiteCpp/SQLiteCpp.h>
#include "observer.hpp"
#include "config/planner_config.hpp"
#include "config/nanowire_config.hpp"
#include "nanowire_system.hpp"
#include <chrono>

namespace acsr {
    using namespace std::chrono_literals;
    template<int STATE_DIMENSION, int CONTROL_DIMENSION>
    class DatabaseObserver
            : public PlannerStartObserver<STATE_DIMENSION,CONTROL_DIMENSION>, public SolutionUpdateObserver<STATE_DIMENSION, CONTROL_DIMENSION> {

        using STATE_TYPE = Eigen::Matrix<double,STATE_DIMENSION,1>;
        using CONTROL_TYPE = Eigen::Matrix<double,CONTROL_DIMENSION,1>;

    public:
        /***
         * constructor. Initial database
         */
        DatabaseObserver(const NanowireSystem<STATE_DIMENSION/2,CONTROL_DIMENSION>& system) : nanowire_system(system),db("plannerDB.db", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE),
                             SolutionUpdateObserver<STATE_DIMENSION, CONTROL_DIMENSION>(), PlannerStartObserver<STATE_DIMENSION,CONTROL_DIMENSION>() {
            SQLite::Statement query(db, "create table if not exists trajectory ("
                                        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                                        "time TEXT NOT NULL, "
                                        "nanowire_count INTEGER NOT NULL,"
                                        "init_state TEXT NOT NULL,"
                                        "target_state TEXT NOT NULL,"
                                        "type TEXT NOT NULL,"
                                        "bidirection INTEGER NOT NULL,"
                                        "optimization INTEGER NOT NULL,"
                                        "stop_type TEXT NOT NULL,"
                                        "stop_value REAL NOT NULL,"
                                        "goal_radius REAL NOT NULL,"
                                        "step_size REAL,"
                                        "min_steps INTEGER,"
                                        "max_steps INTEGER,"
                                        "sst_delta_near REAL,"
                                        "sst_delta_drain REAL,"
                                        "optimization_distance REAL,"
                                        "dominant_path_count REAL"
                                        "blossom_m INTEGER,"
                                        "blossom_n INTEGER,"
                                        "quality_factor REAL,"
                                        "quality_decrease_factor REAL,"
                                        "search_p REAL"
                                        "field_type TEXT"
                                        "filed_dimension REAL"
                                        "nanowire_height TEXT"
                                        "solution_update_table_name TEXT NOT NULL,"
                                        "nodes_update_table_name TEXT NOT NULL"
                                        ")");
            try {
                query.exec();
            } catch (SQLite::Exception &e) {
                std::cout << "Create Table Error\n";
                std::cout << e.what();
                return;
            }
        }

        /***
         * destructor
         */
        ~DatabaseObserver() {

        }

        virtual void onPlannerStart(
                std::string type,
                int robot_count,
                const STATE_TYPE& init_state,
                const STATE_TYPE& target_state,
                const ACADO::VariablesGrid& reference_path,
                const std::string& image_name
        ) override {
            start_time = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(start_time);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%m_%d_%H_%M_%S");
            auto start_time_string = ss.str();

            SQLite::Statement query(db, "INSERT INTO trajectory ("
                                        "time, "
                                        "nanowire_count,"
                                        "init_state,"
                                        "target_state,"
                                        "type,"
                                        "bidirection,"
                                        "optimization,"
                                        "stop_type,"
                                        "stop_value,"
                                        "goal_radius,"
                                        "step_size,"
                                        "min_steps,"
                                        "max_steps,"
                                        "sst_delta_near,"
                                        "sst_delta_drain,"
                                        "optimization_distance,"
                                        "dominant_path_count"
                                        "blossom_m,"
                                        "blossom_n,"
                                        "quality_factor,"
                                        "quality_decrease_factor,"
                                        "search_p"
                                        "field_type"
                                        "filed_dimension"
                                        "nanowire_height"
                                        "solution_update_table_name,"
                                        "nodes_update_table_name"
                                        "VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");

            query.bind(1, start_time_string);
            query.bind(2, robot_count);

            std::string init_string = "";
            std::string target_string = "";
            for (auto i = 0; i < STATE_DIMENSION; ++i) {
                init_string = init_string + std::to_string(init_state(i)) + " ";
            }
            for (auto i = 0; i < STATE_DIMENSION; ++i) {
                target_string = target_string + std::to_string(target_state(i)) + " ";
            }
            query.bind(3, init_string);
            query.bind(4, target_string);
            query.bind(5,type);

            query.bind(6, int(PlannerConfig::bidirection));
            query.bind(7, int(PlannerConfig::optimization));
            query.bind(8, "time");
            query.bind(9, PlannerConfig::total_time);
            query.bind(10, PlannerConfig::goal_radius);
            query.bind(11, PlannerConfig::integration_step);
            query.bind(12, PlannerConfig::min_time_steps);
            query.bind(13, PlannerConfig::max_time_steps);
            query.bind(14, PlannerConfig::sst_delta_near);
            query.bind(15, PlannerConfig::sst_delta_drain);
            query.bind(16, PlannerConfig::optimization_distance);
            query.bind(17, PlannerConfig::dominant_path_count);
            query.bind(18, PlannerConfig::blossomM);
            query.bind(19, PlannerConfig::blossomN);
            query.bind(20, PlannerConfig::quality_factor);
            query.bind(21, PlannerConfig::quality_decrease_factor);
            query.bind(22,PlannerConfig::search_p);
            query.bind(23,NanowireConfig::type);
            query.bind(24,nanowire_system->getFieldDimension());

            auto height = nanowire_system->getNanowireHeight();
            std::string height_string = "";
            for (auto i = 0; i < STATE_DIMENSION/2; ++i) {
                height_string = height_string + std::to_string(height(i)) + " ";
            }

            query.bind(25,height_string);

            solution_table = "solution_" + start_time_string;
            std::string node_table = start_time_string + "_node";

            query.bind(26, solution_table);
            query.bind(27, node_table);

            try {
                query.exec();
            } catch (SQLite::Exception &e) {
                std::cout << "Insert Table Error\n";
                std::cout << e.what();
                return;
            }
            std::this_thread::sleep_for(1s);

            std::string str = "CREATE TABLE  " + solution_table +
                              "  (id INTEGER PRIMARY KEY AUTOINCREMENT, time real, cost real, table_name TEXT)";

            try {
                db.exec(str);
            } catch (SQLite::Exception &e) {
                std::cout << "Create Solution Table Error\n";
                std::cout << e.what();
                return;
            }
        }


        virtual void onSolutionUpdate(const std::vector<STATE_TYPE> &forward_states,
                                      const std::vector<STATE_TYPE> &reverse_states,
                                      const std::vector<STATE_TYPE> &connect_states,
                                      const std::vector<CONTROL_TYPE> &forward_control,
                                      const std::vector<CONTROL_TYPE> &reverse_control,
                                      const std::vector<CONTROL_TYPE> &connect_control,
                                      const std::vector<double> &forward_durations,
                                      const std::vector<double> &reverse_durations,
                                      const std::vector<double> &connect_durations,const std::string& solution_string) override {


            auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now() - start_time).count();
            std::string trajectory_table_name = solution_table + "_time_" + std::to_string(stamp);

            std::string query_string = "INSERT INTO " + solution_table + " (time,cost, table_name) VALUES(?,?,?)";
            SQLite::Statement query(db, query_string);

            double total_cost = std::accumulate(forward_durations.begin(), forward_durations.end(), 0.0);
            total_cost = std::accumulate(reverse_durations.begin(), reverse_durations.end(), total_cost);
            total_cost = std::accumulate(connect_durations.begin(), connect_durations.end(), total_cost);

            query.bind(1, (double) stamp / 1000.0);
            query.bind(2, total_cost);
            query.bind(3, trajectory_table_name);

            try {
                query.exec();
            } catch (SQLite::Exception &e) {
                std::cout << "Insert Solution Error\n";
                std::cout << e.what();
                return;
            }
        }

        void setNanowireSystem(const NanowireSystem<STATE_DIMENSION/2,CONTROL_DIMENSION>& system){
            nanowire_system=system;
        }

    protected:
        SQLite::Database db;
        std::chrono::system_clock::time_point start_time;
        std::string solution_table;
        NanowireSystem<STATE_DIMENSION/2,CONTROL_DIMENSION> nanowire_system;
    };
}
#endif //NANOWIREPLANNER_DATABASE_OBSERVER_HPP
