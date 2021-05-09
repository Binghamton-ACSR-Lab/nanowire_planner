//
// Created by acsr on 5/1/21.
//

#ifndef NANOWIREPLANNER_HTTP_OBSERVER_HPP
#define NANOWIREPLANNER_HTTP_OBSERVER_HPP


#include "observer.hpp"
#include <chrono>
#include <iomanip>
#include <utility>
#include <brynet/net/http/HttpService.hpp>
#include <brynet/net/http/HttpFormat.hpp>
#include <brynet/net/http/WebSocketFormat.hpp>
#include <brynet/net/wrapper/ServiceBuilder.hpp>
#include <brynet/net/wrapper/HttpServiceBuilder.hpp>
#include <brynet/base/AppStatus.hpp>

namespace acsr {
    using namespace std::chrono;
    using namespace brynet;
    using namespace brynet::net;

/***
 * @brief an http server for observing planner data. implement planner start and solution update interface
 */
    class HttpServer : public SolutionUpdateObserver, public PlannerStartObserver {
        //using VectorStateType = Eigen::Matrix<StateType,Eigen::Dynamic,1>;
        //using VectorControlType = Eigen::Matrix<ControlType,Eigen::Dynamic,1>;
    public:
        /***
         * constructor
         * @param _port
         */
        HttpServer(int port_) : port(port_), SolutionUpdateObserver(),
                                PlannerStartObserver() {

        }

        /***
         * implement planner start interface
         * @param type
         * @param robot_count
         * @param _start_state
         * @param _target_state
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
                const Eigen::VectorXd &_start_state,
                const Eigen::VectorXd &_target_state,
                const ACADO::VariablesGrid& reference_path,
                bool bidrectional,
                bool optimization,
                //std::string stop_type,
                double stop_value,
                double goal_radius,
                double step_size,
                int min_steps, int max_steps, double sst_delta_near, double sst_delta_drain,
                double optimization_distance,
                int m, int n, int a, double b,
                const std::string& image_name
        ) override {
            start_time = system_clock::now();
            update_history.clear();
            parameter_table.clear();
            solution_update_table.clear();
            last_solution_table.clear();

            auto in_time_t = std::chrono::system_clock::to_time_t(start_time);

            //first table initializing, containing planner informating
            std::stringstream ss;
            ss << R"(<table>)";

            //1st line, start time
            ss << R"( <tr><td bgcolor = "#66CCFF" >Start Time</td><td align = "right"> )";
            ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
            ss << R"(</td></tr>)";

            //2nd line, planner type
            ss << R"( <tr><td bgcolor = "#66CCFF" >Planner Type</td><td align = "right"> )";
            ss << type;
            ss << R"(</td></tr>)";
            ss << R"( </table> )";
            parameter_table = ss.str();
        }

        /***
         * @brief implement solution update observer
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
        virtual void onSolutionUpdate(const std::vector<Eigen::VectorXd> &forward_states,
                                      const std::vector<Eigen::VectorXd> &reverse_states,
                                      const std::vector<Eigen::VectorXd> &connect_states,
                                      const std::vector<Eigen::VectorXd> &forward_control,
                                      const std::vector<Eigen::VectorXd> &reverse_control,
                                      const std::vector<Eigen::VectorXd> &connect_control,
                                      const std::vector<double> &forward_durations,
                                      const std::vector<double> &reverse_durations,
                                      const std::vector<double> &connect_durations) {
            std::stringstream ss;
            double duration = std::accumulate(forward_durations.begin(), forward_durations.end(), 0.0);
            duration = std::accumulate(reverse_durations.begin(), reverse_durations.end(), duration);
            duration = std::accumulate(connect_durations.begin(), connect_durations.end(), duration);

            update_history.push_back({system_clock::now(), duration});
            ss << R"( <table style="width:50%">
                <tr>
                    <th>update time</th>
                    <th>duration</th>
                </tr>
                )";

            for (auto history:update_history) {
                auto in_time_t = std::chrono::system_clock::to_time_t(history.first);
                ss << R"( <tr align = "middle">
                <td>)";
                ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
                ss << R"(</td>
                <td>)";
                ss << history.second;
                ss << R"(</td>
            </tr> )";
            }
            ss << R"(</table>)";

            ///last solution table;
            ss.clear();
            ss << R"( <table style="width:100%">
                <tr>
                )";
            ///table head
            ///state
            for (auto index = 0; index < forward_states[0].size(); ++index) {
                ss << R"( <th> )" << "state " << index << R"( </th>
            )";
            }
            ///control
            for (auto index = 0; index < forward_control[0].size(); ++index) {
                ss << R"( <th> )" << "control " << index << R"( </th>
            )";
            }
            ///duration
            ss << R"( <th> )" << "duration" << R"( </th>)";
            ss << R"(
            </tr align = "middle"> )";

            ///forward tree information
            std::size_t index = 0;
            for (; index < forward_states.size(); ++index) {
                if (index % 2 == 0)
                    ss << R"( <tr bgcolor = "#00CCFF" align = "middle"> )";
                else {
                    ss << R"( <tr bgcolor = "#6644FF" align = "middle"> )";
                }

                for (auto i = 0; i < forward_states[index].size(); ++i) {
                    ss << R"( <td> )" << forward_states[index][i] << R"( </td>
            )";
                }
                //control
                for (auto i = 0; i < forward_control[index].size(); ++i) {
                    ss << R"( <td> )" << forward_control[index][i] << R"( </td>
            )";
                }
                //duration
                ss << R"( <td> )" << forward_durations[index] << R"( </td> )";
                ss << R"(
            </tr> )";
            }

            ///connect tree information
            for (auto temp_index = 0; temp_index < connect_states.size(); ++index, ++temp_index) {
                if (index % 2 == 0)
                    ss << R"( <tr bgcolor = "#00CC00" align = "middle"> )";
                else {
                    ss << R"( <tr bgcolor = "#66CC00" align = "middle"> )";
                }

                for (auto i = 0; i < connect_states[temp_index].size(); ++i) {
                    ss << R"( <td> )" << connect_states[temp_index][i] << R"( </td>
            )";
                }
                ///control
                for (auto i = 0; i < connect_control[temp_index].size(); ++i) {
                    ss << R"( <td> )" << connect_control[temp_index][i] << R"( </td>
            )";
                }
                ///duration
                ss << R"( <td> )" << connect_durations[temp_index] << R"( </td> )";
                ss << R"(
            </tr> )";
            }

            if (index % 2 == 0)
                ss << R"( <tr align = "middle"> )";
            else {
                ss << R"( <tr bgcolor = "#66CCFF" align = "middle"> )";
            }

            for (auto i = 0; i < reverse_states.front().size(); ++i) {
                ss << R"( <td> )" << reverse_states.front()[i] << R"( </td>
            )";
            }

            if (!connect_control.empty()) {
                ///control
                for (auto i = 0; i < connect_control.back().size(); ++i) {
                    ss << R"( <td> )" << connect_control.back()[i] << R"( </td>
            )";
                }
                //duration
                ss << R"( <td> )" << connect_durations.back() << R"( </td> )";
                ss << R"(
            </tr> )";
            } else {
                ///control
                for (auto i = 0; i < reverse_control.front().size(); ++i) {
                    ss << R"( <td> )" << 0.0 << R"( </td>
            )";
                }
                ///duration
                ss << R"( <td> )" << 0 << R"( </td> )";
                ss << R"(
            </tr> )";
            }
            ++index;

            ///reverse tree information
            for (auto temp_index = 1; temp_index < reverse_states.size(); ++index, ++temp_index) {
                if (index % 2 == 0)
                    ss << R"( <tr align = "middle"> )";
                else {
                    ss << R"( <tr bgcolor = "#66CCFF" align = "middle"> )";
                }

                for (auto i = 0; i < reverse_states[temp_index].size(); ++i) {
                    ss << R"( <td> )" << reverse_states[temp_index][i] << R"( </td>
            )";
                }
                ///control
                for (auto i = 0; i < reverse_control[temp_index].size(); ++i) {
                    ss << R"( <td> )" << reverse_control[temp_index][i] << R"( </td>
            )";
                }
                ///duration
                ss << R"( <td> )" << reverse_durations[temp_index] << R"( </td> )";
                ss << R"(
            </tr> )";
            }
            ss << R"( </table> )";
            last_solution_table = ss.str();

            //setData(parameter_table + solution_update_table + last_solution_table);
        }


        /***
         * stop http server
         */
        void stopServer() {
            if (run_flag) {
                run_flag = false;
            }
        }

        /***
         * @brief start a new thread and run http server
         */
        void runServer() {

            if (run_flag) {
                std::cout << "server is running\n";
                return;
            }
            run_flag = true;
            std::thread t([this]() {
                auto service = TcpService::Create();
                service->startWorkerThread(3);

                auto httpEnterCallback = [this](const brynet::net::http::HTTPParser &httpParser,
                                                const brynet::net::http::HttpSession::Ptr &session) {
                    (void) httpParser;
                    brynet::net::http::HttpResponse response;

                    std::string body = R"(
                <!DOCTYPE html>
                <html>
                    <head>
                        <title>Planner Data</title>
                    </head>
                    <body>
                )";
                    std::ifstream ifs("img.svg");
                    svg_element=std::string( (std::istreambuf_iterator<char>(ifs) ),
                                         (std::istreambuf_iterator<char>()    ) );
                    auto it = svg_element.find_first_of("<svg");
                    svg_element.erase(svg_element.begin(),svg_element.begin()+it-1);

                    std::ifstream solution_ifs("img_solution.svg");
                    auto solution_svg=std::string( (std::istreambuf_iterator<char>(solution_ifs) ),
                                             (std::istreambuf_iterator<char>()    ) );
                    auto it1 = solution_svg.find_first_of("<svg");
                    solution_svg.erase(solution_svg.begin(),solution_svg.begin()+it1-1);

                    body = body + parameter_table + solution_update_table + last_solution_table + (Config::show_node?svg_element:"") +solution_svg  +  R"(</body></html>)";
                    response.setBody(body);
                    std::string result = response.getResult();
                    session->send(result.c_str(), result.size(), [session]() {
                        session->postShutdown();
                    });
                };

                auto wsEnterCallback = [](const brynet::net::http::HttpSession::Ptr &httpSession,
                                          brynet::net::http::WebSocketFormat::WebSocketFrameType opcode,
                                          const std::string &payload) {

                    std::string frame;
                    brynet::net::http::WebSocketFormat::wsFrameBuild(payload.c_str(),
                                                                     payload.size(),
                                                                     frame,
                                                                     brynet::net::http::WebSocketFormat::WebSocketFrameType::TEXT_FRAME,
                                                                     true,
                                                                     false);
                    httpSession->send(std::move(frame));

                };

                brynet::net::wrapper::HttpListenerBuilder listenBuilder;
                listenBuilder.WithService(service)
                        .AddSocketProcess({
                                                        [](TcpSocket &socket) {
                                                            socket.setNodelay();
                                                        },
                                                })
                        .WithMaxRecvBufferSize(2048)
                        .WithAddr(false, "0.0.0.0", port)
                        .WithEnterCallback([httpEnterCallback, wsEnterCallback](
                                const brynet::net::http::HttpSession::Ptr &httpSession,
                                brynet::net::http::HttpSessionHandlers &handlers) {
                            handlers.setHttpCallback(httpEnterCallback);
                            handlers.setWSCallback(wsEnterCallback);

                        })
                        .asyncRun();

                while (run_flag) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                service->stopWorkerThread();
            });

            t.detach();
        }

        /***
         * @brief destructor
         */
        ~HttpServer() override {
            run_flag = false;
        }

    private:
        int port;
        std::vector<std::pair<time_point<system_clock, duration<long int, std::ratio<1, 1000000000>>>, double>> update_history;
        time_point<system_clock, duration<long int, std::ratio<1, 1000000000>>> start_time;
        std::string parameter_table = "No data.";
        std::string solution_update_table;
        std::string last_solution_table;
        std::atomic_bool run_flag = false;
        std::string svg_element;
    };
}

#endif //NANOWIREPLANNER_HTTP_OBSERVER_HPP
