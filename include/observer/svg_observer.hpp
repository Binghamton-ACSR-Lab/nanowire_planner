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

    class SvgObserver : public SolutionUpdateObserver, public PlannerStartObserver, public NodeAddedObserver{
    private:
        const std::vector<Color> colors =  { Color::Red, Color::Black, Color::Blue, Color::Fuchsia,
                                             Color::Green, Color::Lime, Color::Orange, Color::Purple, Color::Silver,Color::Brown , Color::Magenta,  Color::Cyan};
        const std::vector<Color> node_colors={Color::Black,Color::Purple,Color::Orange,Color::Red};
        const double zoom = 1.0/3.0;
        std::string directory_name;
        std::shared_ptr <NanowireConfig> _nanowire_config;
        double width,height;
        Document original_image;
        Document solution_image;
        int index = 0; ///updated index
        std::string shared_image_name;
        std::mutex m;
    public:
        /***
         * default constructor
         */
        SvgObserver() =default;

        SvgObserver(const SvgObserver &) = delete;

        SvgObserver &operator=(const SvgObserver &) = delete;

        /***
         * defalut deconstructor
         */
        virtual ~SvgObserver() = default;



        /***
         * set dynamic system params
         * @param config nanowire dynamic system config
         */
        void setNanowireConfig(const std::shared_ptr<NanowireConfig>& config){
            _nanowire_config = config;
            width = 600*(_nanowire_config->getElectrodesCols()-1) *zoom;
            height = 600*(_nanowire_config->getElectrodesRows()-1) *zoom;

        }

        /**
         * this function will be called when planner starts
         * @param type planner type,useless
         * @param robot_count nanowire count
         * @param init_state start state
         * @param target_state target state
         * @param reference_path reference path, none for isst and sst
         * @param bidrectional is bidirection,useless
         * @param optimization useless
         * @param stop_value useless
         * @param goal_radius useless
         * @param step_size useless
         * @param min_steps useless
         * @param max_steps useless
         * @param sst_delta_near useless
         * @param sst_delta_drain useless
         * @param optimization_distance useless
         * @param m useless
         * @param n useless
         * @param a useless
         * @param b useless
         * @param image_name the image name
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
                int m,int n,int a,double b,
                const std::string& image_name
        ) override{
            original_image = createImage();
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
            directory_name = "image/image_" + ss.str();
            boost::filesystem::create_directories(directory_name);
            shared_image_name = image_name;

            ///draw reference path
            for(auto i=0;i<_nanowire_config->getNanowireCount();++i) {
                Stroke stroke(width/100, colors[i]);
                Polyline ref_path(stroke);
                for(auto j=0;j<reference_path.getNumPoints();++j){
                    ref_path << Point(convertStateToImagePoint(reference_path.getVector(j),i));
                }
                original_image << ref_path;
            }
            solution_image = original_image;
            original_image.save(shared_image_name+".svg");
            original_image.save(shared_image_name+"_solution.svg");

        }

        /***
         * this function will be called when solution updated
         * @param forward_states forward path states
         * @param reverse_states reverse path states
         * @param connect_states connection path states
         * @param forward_control forward path controls
         * @param reverse_control reverse path controls
         * @param connect_control connection path controls
         * @param forward_durations forward path cost
         * @param reverse_durations reverse path cost
         * @param connect_durations connection path cost
         */
        virtual void onSolutionUpdate(const std::vector <Eigen::VectorXd> &forward_states,
                                      const std::vector <Eigen::VectorXd> &reverse_states,
                                      const std::vector <Eigen::VectorXd> &connect_states,
                                      const std::vector <Eigen::VectorXd> &forward_control,
                                      const std::vector <Eigen::VectorXd> &reverse_control,
                                      const std::vector <Eigen::VectorXd> &connect_control,
                                      const std::vector<double> &forward_durations,
                                      const std::vector<double> &reverse_durations,
                                      const std::vector<double> &connect_durations,
                                      const std::string& solution_string) override{

            ///copy the electrodes system image
            Document image(solution_image);

            for(auto i=0;i<_nanowire_config->getNanowireCount();++i) {
                ///forward path
                Stroke forward_stroke(2, colors[i]);
                Polyline forward_path(forward_stroke);
                for (auto& state:forward_states) {
                    forward_path << Point(convertStateToImagePoint(state,i));
                }
                image << forward_path;

                ///start point
                Stroke start_point_stroke(2, colors[i]);
                Circle start_circle(convertStateToImagePoint(forward_states.front(),i),
                                    30, Fill(), start_point_stroke);
                image << start_circle;

                ///reverse path
                Stroke reverse_stroke(2, *(colors.rbegin()+i));
                Polyline reverse_path(reverse_stroke);
                for (auto& state:reverse_states) {
                    reverse_path << Point(convertStateToImagePoint(state,i));
                }
                image << reverse_path;
                ///end point
                Stroke end_point_stroke(2, *(colors.rbegin()+i));
                Circle end_circle(convertStateToImagePoint(reverse_states.back(),i),
                                  10, Fill(), end_point_stroke);
                image << end_circle;

                ///connecting path
                Stroke connect_stroke(2, Color(255, 60, 60));
                Polyline connect_path(connect_stroke);
                connect_path << Point(convertStateToImagePoint(forward_states.back(),i));
                if (connect_states.size() >= 2) {
                    for (auto state:connect_states) {
                        connect_path << Point(convertStateToImagePoint(state,i));
                    }
                }
                connect_path << Point(convertStateToImagePoint(reverse_states.front(),i));
                image << connect_path;
            }

            ///print total cost
            auto d1 = std::accumulate(forward_durations.begin(), forward_durations.end(), 0.0);
            auto d2 = std::accumulate(reverse_durations.begin(), reverse_durations.end(), 0.0);
            auto d3 = std::accumulate(connect_durations.begin(), connect_durations.end(), 0.0);
            Text text(Point(100*zoom, 100*zoom), std::to_string(d1 + d2 + d3), Fill(Color::Black),
                            Font(50*zoom), Stroke(1.5, Color::Black));
            image << text;


            image.save(directory_name + "/image_" + std::to_string(index) + ".svg");
            image.save(shared_image_name+"_solution.svg");
            index++;
        }

        /***
         * create the nanowire system image
         * @return svg document
         */
        Document createImage()  {
            Dimensions dimensions(1.1 * width, 1.1 * height);
            Layout layout(dimensions, Layout::BottomLeft);
            Document svg(layout);
            ///draw boarder
            Polygon border(Stroke(1, Color::Black));
            border << Point(0, 0) << Point(dimensions.width, 0)
                   << Point(dimensions.width, dimensions.height) << Point(0, dimensions.height);
            svg << border;
            ///draw cell edge
            double col_space = width / (_nanowire_config->getElectrodesCols() - 1);
            double row_space = height / (_nanowire_config->getElectrodesRows() - 1);
            Stroke stroke(width/400, Color::Black);
            ///horizontal edges
            for (auto i = 0; i < _nanowire_config->getElectrodesRows(); ++i) {
                Line line(Point(0.05 * width, 0.05 * height + i * row_space),
                          Point(1.05 * width, 0.05 * height + i * row_space), stroke);
                svg << line;
            }
            /// vertical edges
            for (auto i = 0; i < _nanowire_config->getElectrodesCols(); ++i) {
                Line line(Point(0.05 * width + i * col_space, 0.05 * height),
                          Point(0.05 * width + i * col_space, 1.05 * height),
                          stroke);
                svg << line;
            }
            ///draw electrodes
            for (auto i = 0; i < _nanowire_config->getElectrodesRows(); ++i) {
                for (auto j = 0; j < _nanowire_config->getElectrodesCols(); ++j) {
                    Circle circle(
                            Point(0.05 * width + j * col_space, 0.05 * height + i * row_space),
                            width/20, Fill(Color::Green));
                    svg << circle;
                }
            }
            return svg;
        }

        void onNodeAdded(const Eigen::VectorXd &state,TreeId id) override {
            std::scoped_lock<std::mutex> lock(m);
            if(id==TreeId::forward) {
                for(auto i=0;i<state.size()/2;++i) {
                    Circle circle(convertStateToImagePoint(state, i),2, Fill(colors[i]));
                    original_image<<circle;
                }
            }else if(id==TreeId::reverse){
                for(auto i=0;i<state.size()/2;++i) {
                    Circle circle(convertStateToImagePoint(state, i),2, Fill(*(colors.rbegin()+i)));
                    original_image<<circle;
                }
            }
        }

        /***
         * convert the nanowire system state to point in svg image
         * @param state nanowire system state
         * @param robot_index nanowire index
         * @return
         */
        inline Point convertStateToImagePoint(const Eigen::VectorXd &state, int robot_index) {
            return Point(
                    zoom * (state(robot_index * 2)*600/_nanowire_config->getColumnSpace() ) +
                    0.05 * width,
                    zoom * (state(robot_index * 2 + 1)*600/_nanowire_config->getColumnSpace() ) +
                    0.05 * height
            );
        }

        void update(){
            original_image.save(shared_image_name+".svg");
        }


    };
}


#endif //NANOWIREPLANNER_SVG_OBSERVER_HPP
