//
// Created by acsr on 4/30/21.
//

#ifndef NANOWIREPLANNER_RUN_PLANNER_HPP
#define NANOWIREPLANNER_RUN_PLANNER_HPP
#include "global_path.hpp"
#include "nanowire_sst.hpp"
#include "nanowire_isst.hpp"
#include "nanowire_ref_isst.hpp"
#include "config/nanowire_config.hpp"
#include "config/system_config.hpp"
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
        /***
         * default constructor
         */
        RunPlanner() {
        };

        /***
         * necessary initialization
         */
        void init(){
            //read config files
            PlannerConfig::readFile("config/planner.cfg");
            SystemConfig::readFile("config/system.cfg");
            NanowireConfig::readFile("config/nanowire.cfg");

            //callback function for tcp server and http server, to parse string to command from clients
            auto callback  = std::bind(&RunPlanner::parseCommand,this,std::placeholders::_1,std::placeholders::_2);

            // start tcp server
            tcp_server = std::make_shared<TcpServer> (SystemConfig::tcp_port);
            std::cout<<"start tcp server at port "<<SystemConfig::tcp_port<<std::endl;
            tcp_server->run(callback);

            ///start http server
            http_observer = std::make_shared<HttpServer>(SystemConfig::http_port);
            std::cout<<"start http server at port "<<SystemConfig::http_port<<std::endl;
            svg_observer = std::make_shared<SvgObserver>();
            http_observer->run(callback);

            ///set tcp_server as message displayer
            message_displayers.push_back(tcp_server);
        }

        /***
         * performance test
         * @param wire_count nanowire count
         */
        void performanceTest(int wire_count) {
            nanowire_system = std::make_shared<NanowireSystem>(wire_count,_field_dimension);
            http_observer->reset();
            planner = PlannerBuilder::create(PlannerConfig::planner,nanowire_system);

            ///necessary setting
            planner->setStartState(PlannerConfig::init_state);
            planner->setTargetState(PlannerConfig::goal_state);
            planner->setGoalRadius(PlannerConfig::goal_radius);
            //planner->setRandomSeed(time(NULL));

            svg_observer->setNanowireConfig(wire_count);

            planner->registerSolutionUpdateObserver(svg_observer);
            planner->registerPlannerStartObserver(svg_observer);
            planner->registerSolutionUpdateObserver(http_observer);
            planner->registerPlannerStartObserver(http_observer);
            planner->registerNodeAddedObserver(svg_observer);
            planner->setup();
            std::cout<<"Goal Radius: "<<PlannerConfig::goal_radius<<std::endl;
            std::thread forward_thread,reverse_thread,connecting_thread;

            ///forward step threadcc
            forward_run_flag = true;
            forward_thread = std::thread(&RunPlanner::forwardExplore,this);

            ///reverse step thread
            if(PlannerConfig::bidirection){
                reverse_run_flag = true;
                planner->setBiTreePlanner(true);
                reverse_thread = std::thread(&RunPlanner::reverseExplore,this);
            }

            ///connecting thread
            if(PlannerConfig::optimization){
                optimize_run_flag = true;
                planner->setOptimizedConnect(true);
                connecting_thread = std::thread(&RunPlanner::connecting,this);
            }

            auto start = std::chrono::steady_clock::now();

            VariablesGrid vg;
            vg.read("reference_path.txt");
            planner->notifyPlannerStart(getPlannerString(PlannerConfig::planner),"img",vg);
            while(!exit_flag){
                std::cout<<"planner is running\n Number of Node: ";
                auto nodes_count = planner->getNumberOfNode();
                std::cout<<nodes_count.first << "\t"<<nodes_count.second<<"\n";
                std::cout<<"Solution Quality: "<<planner->getBestCost()<<'\n';
                std::cout.flush();
                auto stop = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop-start);
                std::cout<<"duration: " << duration.count()<<"\n";
                if(duration.count()>PlannerConfig::total_time){
                    exit_flag = true;
                    planner->stop();
                    break;
                }
                svg_observer->update();
                std::this_thread::sleep_for (std::chrono::seconds(5));
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
            if(PlannerConfig::optimization){
                if(connecting_thread.joinable())
                    connecting_thread.join();
            }
            std::cout<<"Planner Finished\n";
        }

        /***
         * start and linsten command from server
         */
        void run(){
            ///waiting until receiving exit command
            while(!exit_flag){
                if(!run_flag) {
                    showMessage("waiting command");
                }else{
                    std::cout<<"planner is running\n Number of Node: ";
                    auto nodes_count = planner->getNumberOfNode();
                    std::cout<<nodes_count.first << "\t"<<nodes_count.second<<"\n";
                    if(planner->getBestCost()>1e6){
                        std::cout<<"no solution\n";
                    }else {
                        std::cout << "Solution Quality: " << planner->getBestCost() << '\n';
                    }
                    std::cout.flush();
                    ///to update svg for node, this image can be view via http
                    svg_observer->update();
                }
                std::this_thread::sleep_for (std::chrono::seconds(5));
            }
        }

        /***
         * parse command
         * @param buffer command string
         * @param size command string size
         * @return
         */
        std::string parseCommand(const char* buffer,size_t size){
            std::string s(buffer,size);
            showMessage("Receive Command:\n\t" + s);
            std::string msg;
            boost::to_upper(s); /// to upper case
            boost::trim(s);
            std::vector<std::string> strs;
            ///split command string
            boost::split(strs, s, boost::is_any_of("\t\n, "),boost::token_compress_on);

            if(strs[0]=="START"){
                if(run_flag){
                    msg = "planner is running!";
                    showMessage(msg);
                    return msg;
                }
                ///make sure 2 args at least
                if(strs.size()<2){
                    msg = "Wrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }

                ///nanowire count
                try {
                    _n_wires = std::stoi(strs[1]);
                }catch (std::invalid_argument& e){
                    msg = std::string(e.what()) + "\nWrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }
                ///start wire_count field_dimension zeta_1 ... zeta_2n init_1 ... init_2n target_1 .. target_2n height_1 ... height_n
                if(_n_wires == 0 || strs.size()!=7*_n_wires+3){
                    msg = "Wrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }

                zeta.resize(2*_n_wires);
                _init_states.resize(2*_n_wires);
                _target_states.resize(2*_n_wires);
                _height.resize(_n_wires);

                try {
                    _field_dimension = std::stoi(strs[2]);
                    for (int i=0;i<2*_n_wires;i++) {
                        zeta[i] = std::stod(strs[3 + i]);
                        _init_states[i] = std::stod(strs[3 + 2 * _n_wires + i]) * 1e-6;
                        _target_states[i] = std::stod(strs[3 + 4 * _n_wires + i]) * 1e-6;
                    }
                    for (int i=0;i<_n_wires;i++) {
                        _height[i] = std::stod(strs[3 + 6 * _n_wires + i])* 1e-6;
                    }
                }catch(std::invalid_argument& e){
                    msg = std::string(e.what()) + "\nWrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }

                msg ="";
                {
                    std::vector<std::string> msgs{"Starting new planner: "};
                    msgs.push_back("nano wire count: " + std::to_string(_n_wires));
                    for (int i = 0; i < _n_wires; i++) {
                        msgs.push_back("--Nano Wire " + std::to_string(i + 1) + ": start point: " +
                                       std::to_string(int(_init_states[2 * i]*1e6)) + "," +
                                       std::to_string(int(_init_states[2 * i + 1]*1e6))
                                       + "; destination point: " + std::to_string(int(_target_states[2 * i]*1e6)) + "," +
                                       std::to_string(int(_target_states[2 * i + 1]*1e6)) + ": current height: " + std::to_string(int(_height[i]*1e6)));
                    }
                    std::string str = "--Zeta Potential: ";
                    for (int i = 0; i < 2 * _n_wires; i++) {
                        str.append(std::to_string(zeta[i]) + " ");
                    }
                    msgs.push_back(str);
                    for (auto &s:msgs)
                        msg += s+'\n';
                    showMessage(msg);
                }

                std::thread t(&RunPlanner::startNewPlanner,this);
                t.detach();
                return msg;
            }else if(strs[0]=="RESET"){
                if(!run_flag){
                    msg = "planner not start!";
                    showMessage(msg);
                    return msg;
                }
                if(strs.size()==1){
                    msg = "Wrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }
                int n_wire;
                try {
                    n_wire = std::stoi(strs[1]);
                }catch (std::invalid_argument& e){
                    msg = std::string(e.what()) + "\nWrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }
                if(_n_wires!=n_wire){
                    msg = "Wrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }
                ///reset wire_count field_dimension zeta_1 ... zeta_2n init_1 ... init_2n height_1 ... height_n
                if(strs.size()!=5*_n_wires+3){
                    msg = "Wrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }

                std::vector<double> temp_zeta(2*_n_wires);
                std::vector<double> temp_init_states(2*_n_wires);
                std::vector<double> temp_height(_n_wires);

                try {
                    _field_dimension = std::stoi(strs[2]);
                    for (int i=0;i<2*_n_wires;i++) {
                        temp_zeta[i] = std::stod(strs[3 + i]);
                        temp_init_states[i] = std::stod(strs[3 + 2 * _n_wires + i]) * 1e-6;
                    }
                    for (int i=0;i<_n_wires;i++) {
                        temp_height[i] = std::stod(strs[3 + 4 * _n_wires + i])* 1e-6;
                    }
                }catch(std::invalid_argument& e){
                    msg = std::string(e.what()) + "\nWrong Command Format!!";
                    showMessage(msg);
                    return msg;
                }

                bool zeta_flag = true;
                for (int i=0;i<2*_n_wires;i++) {
                    if(std::abs(temp_zeta[i]-zeta[i])>0.01){
                        zeta_flag = false;
                        break;
                    }
                }

                bool init_flag = true;
                for (int i=0;i<2*_n_wires;i++) {
                    if(std::abs(temp_init_states[i]-_init_states[i])>10e-6){
                        init_flag = false;
                        break;
                    }
                }

                bool height_flag = true;
                for (int i=0;i<_n_wires;i++) {
                    if(std::abs(temp_init_states[i]-_init_states[i])>1e-6){
                        init_flag = false;
                        break;
                    }
                }

                if(zeta_flag & init_flag & height_flag){
                    msg = "reset parameters keeps the same! Ignored\n";
                    showMessage("reset parameters keeps the same! Ignored");
                    return msg;
                }

                _init_states = temp_init_states;
                zeta=temp_zeta;
                _height=temp_height;

                msg = "";
                {
                    std::vector<std::string> msgs{"Reset new planner: "};
                    msgs.push_back("nano wire count: " + std::to_string(_n_wires));
                    for (int i = 0; i < _n_wires; i++) {
                        msgs.push_back("--Nano Wire " + std::to_string(i + 1) + ": start point: " +
                                       std::to_string(int(_init_states[2 * i]*1e6)) + "," +
                                       std::to_string(int(_init_states[2 * i + 1]*1e6)) + ": current height: " + std::to_string(int(_height[i]*1e6)));
                    }
                    std::string str = "Reset:\n--Zeta Potential: ";
                    for (int i = 0; i < 2 * _n_wires; i++) {
                        str.append(std::to_string(zeta[i]) + " ");
                    }
                    msgs.push_back(str);

                    for (auto &s:msgs)
                        msg += s+'\n';
                    showMessage(msg);
                }

                stop();
                ///wait until all threads terminated
                while(!forward_stopped_flag);
                while(!reverse_stopped_flag);
                while(!optimize_stopped_flag);
                std::thread t(&RunPlanner::resetPlanner,this);
                t.detach();
                return msg;
            }else if(strs[0]=="STOP"){
                stop();
                return "planner stopped";
            }else if(strs[0]=="EXIT"){
                exit();
                return "program exit";
            }
            else{
                showMessage("Receive Wrong Command!");
                return "Receive Wrong Command!";
            }
        }

        ~RunPlanner(){
            tcp_server->stopServer();
            http_observer->stopServer();
        }

    private:
        /***
         * forward explore thread
         */
        void forwardExplore(){
            while(!forward_stopped_flag){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            while(forward_run_flag){
                planner->forwardStep();
                forward_stopped_flag = false;
            }
            forward_stopped_flag = true;
        }

        /***
         * reverse explore thread
         */
        void reverseExplore(){
            while(!reverse_stopped_flag){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            while(reverse_run_flag){
                planner->backwardStep();
                reverse_stopped_flag = false;
            }
            reverse_stopped_flag = true;
        }

        /***
         * connect process thread
         */
        void connecting(){
            if(!planner)return;
            while(!optimize_stopped_flag){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            while(optimize_run_flag){
                planner->connectingStep();
                optimize_stopped_flag = false;
            }
            optimize_stopped_flag = true;
        }

        /***
         * notify all planning thread to stop
         */
        void stop(){
            if(planner)planner->stop();
            run_flag = false;
            optimize_run_flag = false;
            forward_run_flag = false;
            reverse_run_flag = false;
        }

        /***
         * notify program to exit
         */
        void exit(){
            stop();
            exit_flag = true;
        }

        /***
         * start a new planner
         */
        void startNewPlanner(){
            if(planner && run_flag){
                std::cout<<"waiting planner to reset!\n";
                showMessage("waiting planner to reset!");
                planner->stop();
            }
            ///reset information on http server.
            http_observer->reset();

            ///create a new nanowire system
            nanowire_system = std::make_shared<NanowireSystem>(_n_wires,_field_dimension);
            ///set parameters for nanowire system
            Eigen::Map<Eigen::VectorXd> height_vec(_height.data(),_n_wires);
            Eigen::Map<Eigen::VectorXd> zeta_vec(zeta.data(),2*_n_wires);
            nanowire_system->init(zeta_vec, height_vec);
            nanowire_system->reset();

            ///create a new planner
            planner = PlannerBuilder::create(PlannerConfig::planner,nanowire_system);
            planner->setStartState(Eigen::Map<Eigen::VectorXd>(_init_states.data(),2*_n_wires));
            planner->setTargetState(Eigen::Map<Eigen::VectorXd>(_target_states.data(),2*_n_wires));
            planner->setGoalRadius(PlannerConfig::goal_radius);


            svg_observer->setNanowireConfig(_n_wires);
            planner->registerSolutionUpdateObserver(svg_observer);
            planner->registerPlannerStartObserver(svg_observer);
            planner->registerSolutionUpdateObserver(http_observer);
            planner->registerPlannerStartObserver(http_observer);
            planner->registerNodeAddedObserver(svg_observer);
            planner->registerSolutionUpdateObserver(tcp_server);

            planner->setup();
            //std::cout<<"Goal Radius: "<<Config::goal_radius<<std::endl;

            std::thread forward_thread,reverse_thread,connecting_thread;

            ///forward step threadcc
            forward_run_flag = true;
            forward_thread = std::thread(&RunPlanner::forwardExplore,this);
            forward_thread.detach();
            if(PlannerConfig::bidirection){
                reverse_run_flag = true;
                planner->setBiTreePlanner(true);
                reverse_thread = std::thread(&RunPlanner::reverseExplore,this);
                reverse_thread.detach();
            }

            if(PlannerConfig::optimization){
                optimize_run_flag = true;
                planner->setOptimizedConnect(true);
                connecting_thread = std::thread(&RunPlanner::connecting,this);
                connecting_thread.detach();
            }
            VariablesGrid vg;
            vg.read("reference_path.txt");
            run_flag = true;
            planner->notifyPlannerStart(getPlannerString(PlannerConfig::planner),"img",vg);

        }

        /***
         * reset planner, keep nanowire system, create new planner
         */
        void resetPlanner(){
            if(planner && run_flag){
                std::cout<<"waiting planner to reset!\n";
                showMessage("waiting planner to reset!");
                planner->stop();
            }
            http_observer->reset();

            ///reset nanowire parameters
            Eigen::Map<Eigen::VectorXd> height_vec(_height.data(),_n_wires);
            Eigen::Map<Eigen::VectorXd> zeta_vec(zeta.data(),2*_n_wires);
            nanowire_system->init(zeta_vec, height_vec);
            nanowire_system->reset();

            ///create new planner
            planner = PlannerBuilder::create(PlannerConfig::planner,nanowire_system);
            planner->setStartState(Eigen::Map<Eigen::VectorXd>(_init_states.data(),2*_n_wires));
            planner->setTargetState(Eigen::Map<Eigen::VectorXd>(_target_states.data(),2*_n_wires));
            planner->setGoalRadius(PlannerConfig::goal_radius);

            ///register observers
            planner->registerSolutionUpdateObserver(svg_observer);
            planner->registerPlannerStartObserver(svg_observer);
            planner->registerSolutionUpdateObserver(http_observer);
            planner->registerPlannerStartObserver(http_observer);
            planner->registerNodeAddedObserver(svg_observer);
            planner->registerSolutionUpdateObserver(tcp_server);

            planner->setup();

            std::thread forward_thread,reverse_thread,connecting_thread;
            ///forward step thread
            forward_run_flag = true;
            forward_thread = std::thread(&RunPlanner::forwardExplore,this);
            forward_thread.detach();
            ///reverse thread
            if(PlannerConfig::bidirection){
                reverse_run_flag = true;
                planner->setBiTreePlanner(true);
                reverse_thread = std::thread(&RunPlanner::reverseExplore,this);
                reverse_thread.detach();
            }
            ///connect thread
            if(PlannerConfig::optimization){
                optimize_run_flag = true;
                planner->setOptimizedConnect(true);
                connecting_thread = std::thread(&RunPlanner::connecting,this);
                connecting_thread.detach();
            }
            run_flag = true;

            VariablesGrid vg;
            vg.read("reference_path.txt");
            planner->notifyPlannerStart(getPlannerString(PlannerConfig::planner),"img",vg);
        }


    private:
        std::shared_ptr<Planner> planner; //planner
        std::shared_ptr<NanowireSystem> nanowire_system; //nanowire system

        ///running flags
        std::atomic_bool forward_run_flag{false};
        std::atomic_bool reverse_run_flag{false};
        std::atomic_bool optimize_run_flag{false};
        //run flag to check the planner is started
        std::atomic<bool> run_flag{false};

        ///stop flags, to check whether the threads are terminated.
        /// note run_flag and stopped_flag could be both false
        std::atomic_bool forward_stopped_flag{true};
        std::atomic_bool reverse_stopped_flag{true};
        std::atomic_bool optimize_stopped_flag{true};
        std::atomic<bool> exit_flag{false};

        ///observers
        std::shared_ptr<SvgObserver> svg_observer;
        std::shared_ptr<HttpServer> http_observer;
        //std::shared_ptr<DatabaseObserver> db_observer;

        int _n_wires;
        int _field_dimension;
        std::vector<double> zeta;
        std::vector<double> _init_states;
        std::vector<double> _target_states;
        std::vector<double> _height;
        std::vector<std::shared_ptr<MessageDisplayer>> message_displayers;
        std::shared_ptr<TcpServer> tcp_server;

    private:
        /***
         * show message to displayer
         * @param msg
         */
        void showMessage(const std::string& msg){
            std::cout<<msg<<'\n';
            for(auto& d:message_displayers)
                d->displayMessage(msg);
        }
    };

}



#endif //NANOWIREPLANNER_RUN_PLANNER_HPP
