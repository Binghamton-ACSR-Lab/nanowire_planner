//
// Created by acsr on 4/30/21.
//

#ifndef NANOWIREPLANNER_SVG_OBSERVER_HPP
#define NANOWIREPLANNER_SVG_OBSERVER_HPP
#include "observer.hpp"
#include "nanowire_system.hpp"
#include <iomanip>
#include <boost/filesystem.hpp>
namespace acsr {

    class SvgObserver : public SolutionUpdateObserver, public PlannerStartObserver {

    private:
        std::string directory_name;
        std::shared_ptr <NanowireSystem> dynamic_system;
        Document original_image;
        size_t index_x;
        size_t index_y;
        int index = 0;
        std::string shared_image_name;

        /*
        cv::Point ConverStateToImagePoint(const Eigen::VectorXd& state){
            auto size = original_image.size();
            return cv::Point(
                    size.width - state(index_x)/dynamic_system->getWidth()*size.width,
                    state(index_y)/dynamic_system->getHeight()*size.height
            );
        }*/

    public:
        SvgObserver(std::shared_ptr <NanowireSystem> system) : dynamic_system(system) {

        }

        SvgObserver(const SvgObserver &) = delete;

        SvgObserver &operator=(const SvgObserver &) = delete;

        virtual ~SvgObserver() = default;

        void setIndexX(size_t index) {
            index_x = index;
        }

        void setIndexY(size_t index) {
            index_y = index;
        }

        virtual void onPlannerStart(
                std::string type,
                int robot_count,
                const Eigen::VectorXd& init_state,
                const Eigen::VectorXd& target_state,
                const ACADO::VariablesGrid& reference_path,
                bool bidrectional,
                bool optimization,
                std::string stop_type,
                double stop_value,
                double goal_radius,
                double step_size,
                int min_steps,int max_steps, double sst_delta_near, double sst_delta_drain,
                double optimization_distance,
                int m,int n,double a,double b,
                const std::string& image_name
        ){
            original_image = dynamic_system->createImage();
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
            directory_name = "image/image_" + ss.str();
            boost::filesystem::create_directories(directory_name);

            index_x = 0;
            index_y = 1;
            shared_image_name = image_name;


        }

        virtual void onSolutionUpdate(const std::vector <Eigen::VectorXd> &forward_states,
                                      const std::vector <Eigen::VectorXd> &reverse_states,
                                      const std::vector <Eigen::VectorXd> &connect_states,
                                      const std::vector <Eigen::VectorXd> &forward_control,
                                      const std::vector <Eigen::VectorXd> &reverse_control,
                                      const std::vector <Eigen::VectorXd> &connect_control,
                                      const std::vector<double> &forward_durations,
                                      const std::vector<double> &reverse_durations,
                                      const std::vector<double> &connect_durations){

            Document image(original_image);

            ///forward path
            Stroke forward_stroke(3, Color(60, 60, 255));
            Polyline forward_path(forward_stroke);
            for (auto state:forward_states) {
                forward_path << Point(dynamic_system->convertStateToImagePoint(state));
            }
            image << forward_path;

            ///start point
            Stroke start_point_stroke(3, Color(60, 60, 255));
            Circle start_circle(dynamic_system->convertStateToImagePoint(forward_states.front()),
                                      30, Fill(), start_point_stroke);
            image << start_circle;

            ///forward path
            Stroke reverse_stroke(3, Color(0, 60, 60));
            Polyline reverse_path(reverse_stroke);
            for (auto state:reverse_states) {
                reverse_path << Point(dynamic_system->convertStateToImagePoint(state));
            }
            image << reverse_path;
            ///end point
            Stroke end_point_stroke(3, Color(0, 60, 60));
            Circle end_circle(dynamic_system->convertStateToImagePoint(reverse_states.back()),
                                    10, Fill(), end_point_stroke);
            image << end_circle;

            ///connecting path
            Stroke connect_stroke(3, Color(255, 60, 60));
            Polyline connect_path(connect_stroke);
            connect_path << Point(dynamic_system->convertStateToImagePoint(forward_states.back()));
            if (connect_states.size() >= 2) {
                for (auto state:connect_states) {
                    connect_path << Point(dynamic_system->convertStateToImagePoint(state));
                }
            }
            connect_path << Point(dynamic_system->convertStateToImagePoint(reverse_states.front()));
            image << connect_path;

            auto d1 = std::accumulate(forward_durations.begin(), forward_durations.end(), 0.0);
            auto d2 = std::accumulate(reverse_durations.begin(), reverse_durations.end(), 0.0);
            auto d3 = std::accumulate(connect_durations.begin(), connect_durations.end(), 0.0);
            //sprintf(text,"%0.2f",d1+d2+d3);
            Text text(Point(200, 200), std::to_string(d1 + d2 + d3), Fill(Color::Black),
                            Font(50), Stroke(2, Color::Black));
            image << text;
            image.save(directory_name + "/image_" + std::to_string(index) + ".svg");
            image.save(shared_image_name+".svg");
            index++;
        }


    };
}


#endif //NANOWIREPLANNER_SVG_OBSERVER_HPP
