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
            svg_observer = std::make_shared<SvgObserver>(nanowire_system);
        };

        void run() {
            //read config files
            auto nanowire_config = std::make_shared<NanowireConfig>();
            nanowire_config->readFile("config/nanowire.cfg");
            Config::readFile("config/planner.cfg");
            nanowire_system = std::make_shared<NanowireSystem>(nanowire_config);
            planner = PlannerBuilder::create(Config::planner,nanowire_system);


///necessary setting
            planner->setStartState(Config::init_state);
            planner->setTargetState(Config::goal_state);
            planner->setGoalRadius(Config::goal_radius);
            //planner->setRandomSeed(time(NULL));



            planner->registerSolutionUpdateObserver(svg_observer);
            planner->registerPlannerStartObserver(svg_observer);
            planner->registerSolutionUpdateObserver(http_observer);
            planner->registerPlannerStartObserver(http_observer);

            planner->setup();

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

            is_running = true;
            while(is_running){

                std::cout<<"planner is running\n Number of Node: ";
                auto nodes_count = planner->getNumberOfNode();
                std::cout<<nodes_count.first << "\t"<<nodes_count.second<<"\n";
                std::cout<<"Solution Quality: "<<planner->getMaxCost()<<'\n';
                std::cout.flush();
                auto stop = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop-start);
                std::cout<<"duration: " << duration.count()<<"\n";
                if(duration.count()>Config::stopping_check){
                    is_running = false;
                    planner->stop();
                    break;
                }
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

        std::atomic_bool is_running;
        ///observers
        std::shared_ptr<SvgObserver> svg_observer;
        //std::shared_ptr<BroadcastObserver> broad_observer;
        std::shared_ptr<HttpServer> http_observer;
        //std::shared_ptr<DatabaseObserver> db_observer;




    };

}



#endif //NANOWIREPLANNER_RUN_PLANNER_HPP
