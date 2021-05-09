//
// Created by acsr on 4/30/21.
//

#ifndef NANOWIREPLANNER_RUN_PLANNER_HPP
#define NANOWIREPLANNER_RUN_PLANNER_HPP
#include "global_path.hpp"
#include "nanowire_sst.hpp"
#include "nanowire_isst.hpp"
#include "nanowire_ref_isst.hpp"
#include "nanowire_config.hpp"
#include "planner_builder.hpp"
#include "svg_observer.hpp"
#include "http_observer.hpp"
#include "tcp_server.hpp"
#include "boost/algorithm/string.hpp"
#include <string>
#include <exception>
namespace acsr {

    class RunPlanner {

    public:
        RunPlanner() {
            forward_run_flag = false;
            reverse_run_flag = false;
            optimize_run_flag = false;

            forward_stopped_flag = true;
            reverse_stopped_flag = true;
            optimize_stopped_flag = true;

            is_running = false;

            ///start http server
            http_observer = std::make_shared<HttpServer>(8080);
            http_observer->runServer();
            svg_observer = std::make_shared<SvgObserver>();
        };

        void init(){
            //read config files
            nanowire_config = std::make_shared<NanowireConfig>();
            nanowire_config->readFile("config/nanowire.cfg");
            Config::readFile("config/planner.cfg");
            nanowire_system = std::make_shared<NanowireSystem>(nanowire_config);
        }

        void initFromTcp(){

        }

        void run() {



            planner = PlannerBuilder::create(Config::planner,nanowire_system);

///necessary setting
            planner->setStartState(Config::init_state);
            planner->setTargetState(Config::goal_state);
            planner->setGoalRadius(Config::goal_radius);
            //planner->setRandomSeed(time(NULL));

            svg_observer->setNanowireConfig(nanowire_config);

            planner->registerSolutionUpdateObserver(svg_observer);
            planner->registerPlannerStartObserver(svg_observer);
            planner->registerSolutionUpdateObserver(http_observer);
            planner->registerPlannerStartObserver(http_observer);
            planner->registerNodeAddedObserver(svg_observer);



            planner->setup();
            std::cout<<"Goal Radius: "<<Config::goal_radius<<std::endl;

            std::thread forward_thread,reverse_thread,connecting_thread;

            ///forward step threadcc
            forward_run_flag = true;
            forward_stopped_flag = false;
            forward_thread = std::thread(&RunPlanner::forwardExplore,this);
            //thread_vector.push_back(std::move(forward_thread));

            ///reverse step thread

            if(Config::bidirection){
                reverse_run_flag = true;
                reverse_stopped_flag = false;
                planner->setBiTreePlanner(true);
                reverse_thread = std::thread(&RunPlanner::reverseExplore,this);

                //reverse_thread.join();
                //thread_vector.push_back(std::move(reverse_thread));
            }

            ///connecting thread
            if(Config::optimization){
                optimize_run_flag = true;
                optimize_stopped_flag = false;
                planner->setOptimizedConnect(true);
                connecting_thread = std::thread(&RunPlanner::connecting,this);
                //thread_vector.push_back(std::move(connecting_thread));
            }

            auto start = std::chrono::steady_clock::now();

            VariablesGrid vg;
            vg.read("reference_path.txt");
            is_running = true;
            planner->notifyPlannerStart(getPlannerString(Config::planner),"img",vg);
            while(is_running){

                std::cout<<"planner is running\n Number of Node: ";
                auto nodes_count = planner->getNumberOfNode();
                std::cout<<nodes_count.first << "\t"<<nodes_count.second<<"\n";
                std::cout<<"Solution Quality: "<<planner->getMaxCost()<<'\n';
                std::cout.flush();
                auto stop = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop-start);
                std::cout<<"duration: " << duration.count()<<"\n";
                if(duration.count()>Config::total_time){
                    is_running = false;
                    planner->stop();
                    break;
                }
                svg_observer->update();
                std::this_thread::sleep_for (std::chrono::seconds(2));
            }

            reverse_run_flag = false;
            forward_run_flag = false;
            optimize_run_flag = false;


            if(forward_thread.joinable())
                forward_thread.join();
            if(planner->isBiTreePlanner()) {
                if(reverse_thread.joinable())
                    reverse_thread.join();
            }
            if(Config::optimization){
                if(connecting_thread.joinable())
                    connecting_thread.join();
            }
            std::cout<<"Planner Finished\n";

        }

        void runByTcp(){
            TcpServer server(6060);
            auto callback  = std::bind(&RunPlanner::parseCommand,this,std::placeholders::_1,std::placeholders::_2);
            server.run(callback);
            while(!stop_flag){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

        }

        void parseCommand(const char* buffer,size_t size){
            std::string s(buffer,size);
            boost::to_upper(s);
            std::string delim = " ";
            auto start = 0U;
            auto end = s.find(delim);
            std::vector<std::string> strs;
            while (end != std::string::npos)
            {
                strs.push_back(s.substr(start, end - start));
                start = end + delim.length();
                end = s.find(delim, start);
            }
            strs.push_back(s.substr(start, end));
            if(strs[0]=="START"){
                if(!stop_flag){
                    std::cout<<"planner is running!\n";
                    return;
                }
                if(strs.size()==1){
                    std::cout<<"Wrong Command Format!!\n";
                    return;
                }

                try {
                    _n_wires = std::stoi(strs[1]);
                }catch (std::invalid_argument& e){
                    std::cout<<e.what()<<"\nWrong Command Format!!\n";
                    return;
                }

                if(_n_wires == 0 || strs.size()!=6*_n_wires+2){
                    std::cout<<"Wrong Command Format!!\n";
                    return;
                }

                zeta.resize(2*_n_wires);
                _init_states.resize(2*_n_wires);
                _target_states.resize(2*_n_wires);
                for (int i=0;i<2*_n_wires;i++) {
                    try {
                        zeta[i] = std::stod(strs[2 + i]);
                        _init_states[i] = std::stod(strs[2 + 2 * _n_wires + i]);
                        _target_states[i] = std::stod(strs[2 + 4 * _n_wires + i]);
                    }catch(std::invalid_argument& e){
                        std::cout<<e.what()<<"\nWrong Command Format!!\n";
                        return;
                    }
                }

                /*
                std::vector<std::string> msgs{"Starting new planner: "};
                msgs.push_back("nano wire count: " + std::to_string(robot_count));
                for (int i = 0; i < robot_count; i++) {
                    msgs.push_back("--Nano Wire " + std::to_string(i + 1) + ": start point: " +
                                   std::to_string(start_states[2 * i]) + "," +
                                   std::to_string(start_states[2 * i + 1])
                                   + "; destination point: " + std::to_string(destination_states[2 * i]) + "," +
                                   std::to_string(destination_states[2 * i + 1]));
                }
                std::string str = "--Zeta Potential: ";
                for (int i = 0; i < 2*robot_count; i++) {
                    str.append( std::to_string(theta[i]) + " ");
                }
                msgs.push_back(str);
                msgs.push_back("electrodes configure: ");
                msgs.push_back(
                        "--rows: " + std::to_string(electrode_rows) + " ---- cols: " +
                        std::to_string(electrode_cols));
                for (auto s:msgs)
                    qInfo()<<QString::fromStdString(s);*/
                stop_flag=false;
            }else if(strs[0]=="RESET"){

            }else if(strs[0]=="STOP"){

            }
            else{
                std::cout<<"Receive Wrong Command!\n";
            }

        }

        ~RunPlanner(){
            //broad_observer->stopServer();
            http_observer->stopServer();
        }

    private:
        /***
     * forward explore
     */
        void forwardExplore(){
            while(forward_run_flag){
                planner->forwardStep();
            }
            forward_stopped_flag = true;
        }

        /***
         * reverse explore
         */
        void reverseExplore(){
            while(reverse_run_flag){
                planner->reverseStep();
            }
            reverse_stopped_flag = true;
        }

        /***
         * connect process
         */
        void connecting(){
            if(!planner)return;
            while(optimize_run_flag){
                planner->connectingStep();
            }
            optimize_stopped_flag = true;
        }

    private:
        std::shared_ptr<Planner> planner;


        ///dynamic system
        std::shared_ptr<NanowireSystem> nanowire_system;


        ///running flags
        bool forward_run_flag;
        bool reverse_run_flag;
        bool optimize_run_flag;

        bool forward_stopped_flag;
        bool reverse_stopped_flag;
        bool optimize_stopped_flag;

        std::atomic<bool> stop_flag{false};

        std::atomic_bool is_running;
        ///observers
        std::shared_ptr<SvgObserver> svg_observer;
        //std::shared_ptr<BroadcastObserver> broad_observer;
        std::shared_ptr<HttpServer> http_observer;
        //std::shared_ptr<DatabaseObserver> db_observer;

        std::shared_ptr<NanowireConfig> nanowire_config;

        int _n_wires;
        std::vector<double> zeta;
        std::vector<double> _init_states;
        std::vector<double> _target_states;





    };

}



#endif //NANOWIREPLANNER_RUN_PLANNER_HPP
