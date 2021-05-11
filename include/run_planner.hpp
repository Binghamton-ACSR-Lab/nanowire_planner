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


            //is_running = false;

            ///start http server

        };

        void init(){
            //read config files
            nanowire_config = std::make_shared<NanowireConfig>();
            nanowire_config->readFile("config/nanowire.cfg");
            Config::readFile("config/planner.cfg");
            nanowire_system = std::make_shared<NanowireSystem>(nanowire_config);

            auto callback  = std::bind(&RunPlanner::parseCommand,this,std::placeholders::_1,std::placeholders::_2);
            tcp_server = std::make_shared<TcpServer> (6060);
            tcp_server->run(callback);

            http_observer = std::make_shared<HttpServer>(8080);
            svg_observer = std::make_shared<SvgObserver>();
            http_observer->run(callback);
            message_displayers.push_back(tcp_server);
        }

        void initFromTcp(){

        }

        void run() {



            http_observer->reset();
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




            planner->setup(nanowire_config);
            std::cout<<"Goal Radius: "<<Config::goal_radius<<std::endl;

            std::thread forward_thread,reverse_thread,connecting_thread;

            ///forward step threadcc
            forward_run_flag = true;
            //forward_stopped_flag = false;
            forward_thread = std::thread(&RunPlanner::forwardExplore,this);
            //thread_vector.push_back(std::move(forward_thread));

            ///reverse step thread

            if(Config::bidirection){
                reverse_run_flag = true;
                //reverse_stopped_flag = false;
                planner->setBiTreePlanner(true);
                reverse_thread = std::thread(&RunPlanner::reverseExplore,this);

                //reverse_thread.join();
                //thread_vector.push_back(std::move(reverse_thread));
            }

            ///connecting thread
            if(Config::optimization){
                optimize_run_flag = true;
                //optimize_stopped_flag = false;
                planner->setOptimizedConnect(true);
                connecting_thread = std::thread(&RunPlanner::connecting,this);
                //thread_vector.push_back(std::move(connecting_thread));
            }

            auto start = std::chrono::steady_clock::now();

            VariablesGrid vg;
            vg.read("reference_path.txt");
            //is_running = true;
            planner->notifyPlannerStart(getPlannerString(Config::planner),"img",vg);
            while(!exit_flag){

                std::cout<<"planner is running\n Number of Node: ";
                auto nodes_count = planner->getNumberOfNode();
                std::cout<<nodes_count.first << "\t"<<nodes_count.second<<"\n";
                std::cout<<"Solution Quality: "<<planner->getMaxCost()<<'\n';
                std::cout.flush();
                auto stop = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop-start);
                std::cout<<"duration: " << duration.count()<<"\n";
                if(duration.count()>Config::total_time){
                    exit_flag = true;
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

            int v = 0;
            while(!exit_flag){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                ++v;
                if(v==1000){
                    v=0;
                    //server.displayMessage("Hello");
                }
            }

        }


        std::string parseCommand(const char* buffer,size_t size){
            std::string s(buffer,size);
            std::string msg;
            boost::to_upper(s);
            boost::trim(s);
            std::vector<std::string> strs;
            //boost::algorithm::split()
            boost::split(strs, s, boost::is_any_of(" "),boost::token_compress_on);
            /*
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
            strs.push_back(s.substr(start, end));*/
            if(strs[0]=="START"){
                if(run_flag){
                    msg = "planner is running!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }
                if(strs.size()==1){
                    msg = "Wrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }

                try {
                    _n_wires = std::stoi(strs[1]);
                }catch (std::invalid_argument& e){
                    msg = std::string(e.what()) + "\nWrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }

                if(_n_wires == 0 || strs.size()!=6*_n_wires+2){
                    msg = "Wrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }

                zeta.resize(2*_n_wires);
                _init_states.resize(2*_n_wires);
                _target_states.resize(2*_n_wires);
                for (int i=0;i<2*_n_wires;i++) {
                    try {
                        zeta[i] = std::stod(strs[2 + i]);
                        _init_states[i] = std::stod(strs[2 + 2 * _n_wires + i])*1e-6;
                        _target_states[i] = std::stod(strs[2 + 4 * _n_wires + i])*1e-6;
                    }catch(std::invalid_argument& e){
                        msg = std::string(e.what()) + "\nWrong Command Format!!";
                        std::cout<<msg<<'\n';
                        showMessage(msg);
                        return msg;
                    }
                }

                msg ="";
                {
                    std::vector<std::string> msgs{"Starting new planner: "};
                    msgs.push_back("nano wire count: " + std::to_string(_n_wires));
                    for (int i = 0; i < _n_wires; i++) {
                        msgs.push_back("--Nano Wire " + std::to_string(i + 1) + ": start point: " +
                                       std::to_string(_init_states[2 * i]) + "," +
                                       std::to_string(_init_states[2 * i + 1])
                                       + "; destination point: " + std::to_string(_target_states[2 * i]) + "," +
                                       std::to_string(_target_states[2 * i + 1]));
                    }
                    std::string str = "--Zeta Potential: ";
                    for (int i = 0; i < 2 * _n_wires; i++) {
                        str.append(std::to_string(zeta[i]) + " ");
                    }
                    msgs.push_back(str);
                    for (auto &s:msgs)
                        msg += s+'\n';
                    std::cout << msg;
                    showMessage(msg);
                }
                //run_flag=true;
                //is_running = true;
                std::thread t(&RunPlanner::startNewPlanner,this);
                t.detach();
                return msg;

            }else if(strs[0]=="RESET"){
                if(!run_flag){
                    msg = "planner not start!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }
                if(strs.size()==1){
                    msg = "Wrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }

                int n_wire;
                try {
                    n_wire = std::stoi(strs[1]);
                }catch (std::invalid_argument& e){
                    msg = std::string(e.what()) + "\nWrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }
                if(_n_wires!=n_wire){
                    msg = "Wrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }

                if(strs.size()!=4*_n_wires+2){
                    msg = "Wrong Command Format!!";
                    std::cout<<msg<<'\n';
                    showMessage(msg);
                    return msg;
                }

                std::vector<double> temp_zeta(2*_n_wires);
                std::vector<double> temp_init_states(2*_n_wires);

                for (int i=0;i<2*_n_wires;i++) {
                    try {
                        temp_zeta[i] = std::stod(strs[2 + i]);
                        temp_init_states[i] = std::stod(strs[2 + 2 * _n_wires + i])*1e-6;
                    }catch(std::invalid_argument& e){
                        msg = std::string(e.what()) + "\nWrong Command Format!!";
                        std::cout<<msg<<'\n';
                        showMessage(msg);
                        return msg;
                    }
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

                if(zeta_flag & init_flag){
                    msg = "reset parameters keeps the same! Ignored\n";
                    std::cout<<msg<<'\n';
                    showMessage("reset parameters keeps the same! Ignored");
                    return msg;
                }

                msg = "";
                {
                    std::vector<std::string> msgs{"Reset new planner: "};
                    msgs.push_back("nano wire count: " + std::to_string(_n_wires));
                    for (int i = 0; i < _n_wires; i++) {
                        msgs.push_back("--Nano Wire " + std::to_string(i + 1) + ": start point: " +
                                       std::to_string(_init_states[2 * i]) + "," +
                                       std::to_string(_init_states[2 * i + 1]));
                    }
                    std::string str = "Reset:\n--Zeta Potential: ";
                    for (int i = 0; i < 2 * _n_wires; i++) {
                        str.append(std::to_string(zeta[i]) + " ");
                    }
                    msgs.push_back(str);

                    for (auto &s:msgs)
                        msg += s+'\n';
                    std::cout << msg;
                    showMessage(msg);

                }

                if(zeta_flag){
                    stop();
                    _init_states = temp_init_states;
                    zeta = temp_zeta;
                    while(!forward_stopped_flag);
                    while(!reverse_stopped_flag);
                    while(!optimize_stopped_flag);
                    std::thread t(&RunPlanner::startNewPlanner,this);
                    t.detach();
                    return msg;
                }else{
                    stop();
                    while(!forward_stopped_flag);
                    while(!reverse_stopped_flag);
                    while(!optimize_stopped_flag);
                    std::thread t(&RunPlanner::startNewPlanner,this);
                    t.detach();
                    return msg;
                }
                //run_flag=true;
                //is_running = true;
                //startNewPlanner();
            }else if(strs[0]=="STOP"){
                stop();
                return "planner stopped";
            }else if(strs[0]=="EXIT"){
                exit();
                return "program exit";
            }
            else{
                std::cout<<"Receive Wrong Command!\n";
                showMessage("Receive Wrong Command!");
                return "Receive Wrong Command!";
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
         * reverse explore
         */
        void reverseExplore(){
            while(!reverse_stopped_flag){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            while(reverse_run_flag){
                planner->reverseStep();
                reverse_stopped_flag = false;
            }
            reverse_stopped_flag = true;
        }

        /***
         * connect process
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

        void stop(){
            if(planner)planner->stop();
            run_flag = false;
            //is_running = false;
            optimize_run_flag = false;
            forward_run_flag = false;
            reverse_run_flag = false;
            notify_flag = false;
        }

        void exit(){
            stop();
            exit_flag = true;
        }


        void startNewPlanner(){
            if(planner && run_flag){
                std::cout<<"waiting planner to reset!\n";
                showMessage("waiting planner to reset!");
                //nanowire_system->stop();
                planner->stop();
            }

            //read config files

            http_observer->reset();

            nanowire_config->readFile("config/nanowire.cfg");
            nanowire_config->setNanowireCount(_n_wires);
            nanowire_config->setZeta(zeta);
            //Config::readFile("config/planner.cfg");
            nanowire_system = std::make_shared<NanowireSystem>(nanowire_config);
            planner = PlannerBuilder::create(Config::planner,nanowire_system);

            planner->setStartState(Eigen::Map<Eigen::VectorXd>(_init_states.data(),2*_n_wires));
            planner->setTargetState(Eigen::Map<Eigen::VectorXd>(_target_states.data(),2*_n_wires));
            planner->setGoalRadius(Config::goal_radius);

            svg_observer->setNanowireConfig(nanowire_config);
            planner->registerSolutionUpdateObserver(svg_observer);
            planner->registerPlannerStartObserver(svg_observer);
            planner->registerSolutionUpdateObserver(http_observer);
            planner->registerPlannerStartObserver(http_observer);
            planner->registerNodeAddedObserver(svg_observer);

            planner->registerSolutionUpdateObserver(tcp_server);

            planner->setup(nanowire_config);
            //std::cout<<"Goal Radius: "<<Config::goal_radius<<std::endl;

            std::thread forward_thread,reverse_thread,connecting_thread;

            ///forward step threadcc
            forward_run_flag = true;
            forward_thread = std::thread(&RunPlanner::forwardExplore,this);
            forward_thread.detach();
            if(Config::bidirection){
                reverse_run_flag = true;
                planner->setBiTreePlanner(true);
                reverse_thread = std::thread(&RunPlanner::reverseExplore,this);
                reverse_thread.detach();
            }

            if(Config::optimization){
                optimize_run_flag = true;
                planner->setOptimizedConnect(true);
                connecting_thread = std::thread(&RunPlanner::connecting,this);
                connecting_thread.detach();
            }
            while(!notify_stopped_flag){}
            notify_flag = true;
            std::thread timer([this](){
                while(notify_flag){
                    std::cout<<"planner is running\n Number of Node: ";
                    auto nodes_count = planner->getNumberOfNode();
                    std::cout<<nodes_count.first << "\t"<<nodes_count.second<<"\n";
                    std::cout<<"Solution Quality: "<<planner->getMaxCost()<<'\n';
                    std::cout.flush();

                    svg_observer->update();
                    std::this_thread::sleep_for (std::chrono::seconds(2));
                    notify_stopped_flag =false;
                }
                notify_stopped_flag = true;

            });
            timer.detach();

            VariablesGrid vg;
            vg.read("reference_path.txt");
            run_flag = true;
            planner->notifyPlannerStart(getPlannerString(Config::planner),"img",vg);

        }

    private:
        std::shared_ptr<Planner> planner;


        ///dynamic system
        std::shared_ptr<NanowireSystem> nanowire_system;


        ///running flags
        std::atomic_bool forward_run_flag{false};
        std::atomic_bool reverse_run_flag{false};
        std::atomic_bool optimize_run_flag{false};

        std::atomic_bool forward_stopped_flag{true};
        std::atomic_bool reverse_stopped_flag{true};
        std::atomic_bool optimize_stopped_flag{true};

        std::atomic<bool> run_flag{false};
        std::atomic<bool> exit_flag{false};

        std::atomic<bool> notify_flag{false};
        std::atomic<bool> notify_stopped_flag{true};

        //std::atomic_bool is_running;
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
        std::vector<std::shared_ptr<MessageDisplayer>> message_displayers;

        std::shared_ptr<TcpServer> tcp_server;

    private:
        void showMessage(const std::string& msg){
            for(auto& d:message_displayers)
                d->displayMessage(msg);
        }

    };

}



#endif //NANOWIREPLANNER_RUN_PLANNER_HPP
