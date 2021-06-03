//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_ISST_HPP
#define NANOWIREPLANNER_NANOWIRE_ISST_HPP
#include "nanowire_sst.hpp"

namespace acsr{

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class iSST: virtual public SST<STATE_DIMENSION,CONTROL_DIMENSION>,virtual public Planner<STATE_DIMENSION,CONTROL_DIMENSION>{
        //using Planner<STATE_DIMENSION>::_dynamic_system;
        //using Planner<STATE_DIMENSION>::_best_goal;
        //using Planner<STATE_DIMENSION>::_init_state;
        //using Planner<STATE_DIMENSION>::_target_state;
        //using Planner<STATE_DIMENSION>::_best_cost;
        //using Planner<STATE_DIMENSION>::_run_flag;
        //using Planner<STATE_DIMENSION>::_planner_connection;
        //using SST<STATE_DIMENSION>::optimize_set;
        //using SST<STATE_DIMENSION>::addToTree;



        using StateType = Eigen::Matrix<double,STATE_DIMENSION,1>;
        using ControlType = Eigen::Matrix<double,CONTROL_DIMENSION,1>;

        using TreeNodeType = TreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>;
        using SSTTreeNodeType = SSTTreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>;

        using NodePtr = std::shared_ptr<Node<double,STATE_DIMENSION>>;
        using TreeNodePtr = std::shared_ptr <TreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>>;
        using SSTTreeNodePtr = std::shared_ptr <SSTTreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>>;


    protected:

        /***
         * setup
         */
        void setup() override {
            SST<STATE_DIMENSION,CONTROL_DIMENSION>::setup();
        }

        /***
         * select a node to explore, unlike in SST or RRT, this process is not stochastic. we select a node with least estimated solution cost
         * @param tree_id
         * @param parent
         * @return
         */
        bool searchSelection(TreeId tree_id, TreeNodePtr& parent){
            auto point = this->_dynamic_system->randomState();
            auto near = this->getNearNodeByCount(point,tree_id,10);
            auto it = std::min_element(near.begin(), near.end(),
                                       [](const NodePtr &node1, const NodePtr &node2) {
                                           return std::static_pointer_cast<TreeNodeType>(node1)->getCost() < std::static_pointer_cast<TreeNodeType>(node2)->getCost();
                                       });
            parent = std::static_pointer_cast<TreeNodeType>(*it);
            if(nullptr == parent)
                return false;
            return true;
        }

        /***
         * forward blossom
         * @param parent the node to be explored
         * @param state store the temp state
         * @param control store the temp control
         * @param duration store temp duration
         * @return true if blossom process success
         */
        bool forwardBlossom(const SSTTreeNodePtr & parent,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>& params){

            if(parent->getTreeId()!=TreeId::forward || !parent->isActive())
                return false;

            const int n=3;
            std::vector<std::thread> thread_pool;
            std::vector<std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>> heuristic_vec(n,{1e6,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>()});
            for(auto i=0;i<n-1;++i){
                thread_pool.push_back(std::thread(&acsr::iSST<STATE_DIMENSION,CONTROL_DIMENSION>::forwardBlossomThread,this,parent,PlannerConfig::blossomM/n,std::ref(heuristic_vec[i])));
            }
            ///propagate M times and select a best node with least estimated solution cost
            StateType temp_state;
            double temp_duration;

            for(int i=0;i<PlannerConfig::blossomM/n;i++) {
                if(!this->_run_flag)return false;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,
                                                                   steps,temp_state,temp_duration)){
                    //auto d = temp_duration + this->_dynamic_system->getHeuristic(temp_state,this->_goal->getState());
                    auto d = this->_dynamic_system->getHeuristic(temp_state,this->_goal->getState());
                    if(d<heuristic_vec[n-1].first){
                        heuristic_vec[n-1].first = d;
                        heuristic_vec[n-1].second = PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> {temp_state,temp_control,temp_duration};
                    }
                }
            }
            for(auto&t:thread_pool){
                if(t.joinable())t.join();
            }

            auto it = std::min_element(heuristic_vec.begin(),heuristic_vec.end(),[](const std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& p1,
                    const std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& p2){
                return p1.first<p2.first;
            });
            if(it->first>0.9e6)return false;
            params=it->second;
            return true;
        }

        /***
         * backward blossom
         * @param parent the node to be explored
         * @param state store the temp state
         * @param control store the temp control
         * @param duration store temp duration
         * @return true if blossom process success
         */
        bool backwardBlossom(const SSTTreeNodePtr& parent,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>& params){
            if(parent->getTreeId()!=TreeId::backward || !parent->isActive())
                return false;
            const int n=3;
            std::vector<std::thread> thread_pool;
            std::vector<std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>> heuristic_vec(n,{1e6,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>()});
            for(auto i=0;i<n-1;++i){
                thread_pool.push_back(std::thread(&acsr::iSST<STATE_DIMENSION,CONTROL_DIMENSION>::backwardBlossomThread,this,parent,PlannerConfig::blossomM/n,std::ref(heuristic_vec[i])));
            }
            ///propagate M times and select a best node with least estimated solution cost
            StateType temp_state;
            double temp_duration;

            for(int i=0;i<PlannerConfig::blossomM/n;i++) {
                if(!this->_run_flag)return false;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,
                                                                   steps,temp_state,temp_duration)){
                    auto d = this->_dynamic_system->getHeuristic(temp_state,this->_goal->getState());
                    if(d<heuristic_vec[n-1].first){
                        heuristic_vec[n-1].first = d;
                        heuristic_vec[n-1].second = PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> {temp_state,temp_control,temp_duration};
                    }
                }
            }
            for(auto&t:thread_pool){
                if(t.joinable())t.join();
            }

            auto it = std::min_element(heuristic_vec.begin(),heuristic_vec.end(),[](const std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& p1,
                                                                                    const std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& p2){
                return p1.first<p2.first;
            });
            if(it->first>0.9e6)return false;
            params=it->second;
            return true;
        }

        void backwardBlossomThread(const TreeNodePtr& parent,int iterate_n,std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& heuristic_map){
            StateType temp_state;
            double temp_duration;
            for(int i=0;i<iterate_n;i++) {
                if(!this->_run_flag)return;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->backwardPropagateBySteps(parent->getState(),temp_control,
                                                                    steps,temp_state,temp_duration)){
                    if(!this->_root){
                        std::cout<<"I am here\n";
                    }
                    auto d = this->_dynamic_system->getHeuristic(temp_state,this->_root->getState());
                    if(d<heuristic_map.first){
                        heuristic_map.first = d;
                        heuristic_map.second = PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> {temp_state,temp_control,temp_duration};
                    }
                }
            }
        }

        void forwardBlossomThread(const TreeNodePtr& parent,int iterate_n,std::pair<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& heuristic_map){
            StateType temp_state;
            double temp_duration;
            for(int i=0;i<iterate_n;i++) {
                if(!this->_run_flag)return;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,
                                                                   steps,temp_state,temp_duration)){
                    auto d = this->_dynamic_system->getHeuristic(temp_state,this->_goal->getState());
                    if(d<heuristic_map.first){
                        heuristic_map.first = d;
                        heuristic_map.second = PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> {temp_state,temp_control,temp_duration};
                    }
                }
            }
        }

    public:
        explicit iSST(std::shared_ptr<NanowireSystem<STATE_DIMENSION/2,16>> dynamic_system):SST<STATE_DIMENSION,CONTROL_DIMENSION>(dynamic_system),
                                                                      Planner<STATE_DIMENSION,CONTROL_DIMENSION>(dynamic_system){
        }

        iSST()=delete;
        iSST(const iSST&) = delete;

        ~iSST() override{

        }

        /***
         * override forward step
         */
        void forwardStep() override{
            ///select a node to be explored in forward tree
            TreeNodePtr parent;
            searchSelection(TreeId::forward,parent);
            if(!std::static_pointer_cast<SSTTreeNodeType>(parent)->isActive())
                return;

            //StateType state;
            //ControlType control;
            //double duration;
            ///cast the node to sst type
            PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> params;

            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNodeType>(parent);

            ///perform blossom, until it fails
            while (forwardBlossom(sst_parent,params)){
                if(sst_parent== nullptr)return;
                auto new_node = this->addToTree(TreeId::forward,sst_parent,params.state,params.control,params.duration);
                this->checkConnection(new_node);
                if(PlannerConfig::show_node && new_node!= nullptr){
                    std::thread t([this,&params](){
                        this->notifyNodeAdded(params.state,TreeId::forward);
                    });
                    t.detach();
                }
                ///new node not exist or new node is the best_goal
                if(new_node == nullptr || this->_best_goal.first == new_node){
                    break;
                }else {
                    sst_parent = new_node;
                }
            }
        }

        /***
         * override backward step
         */
        void backwardStep() override{
            if(!this->_is_bi_tree_planner)
                return;
            TreeNodePtr parent;
            searchSelection(TreeId::backward,parent);
            if(!std::static_pointer_cast<SSTTreeNodeType>(parent)->isActive())return;

            //StateType state;
            //ControlType control;
            //double duration;
            PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> params;
            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNodeType>(parent);
            while (backwardBlossom(sst_parent,params)){
                auto new_node = this->addToTree(TreeId::backward,sst_parent,params.state,params.control,params.duration);
                this->checkConnection(new_node);
                if(PlannerConfig::show_node && new_node!= nullptr){
                    std::thread t([this,&params](){
                        this->notifyNodeAdded(params.state,TreeId::backward);
                    });
                    t.detach();
                }
                if(new_node == nullptr || this->_best_goal.second == new_node){
                    break;
                }else
                    sst_parent=new_node;
            }
        }
    };
}



#endif //NANOWIREPLANNER_NANOWIRE_ISST_HPP
