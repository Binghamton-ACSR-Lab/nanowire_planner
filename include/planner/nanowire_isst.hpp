//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_ISST_HPP
#define NANOWIREPLANNER_NANOWIRE_ISST_HPP
#include "nanowire_sst.hpp"

namespace acsr{

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class iSST: public SST<STATE_DIMENSION,CONTROL_DIMENSION>,virtual public Planner<STATE_DIMENSION,CONTROL_DIMENSION>{
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

        using TreeNodeType = TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>;
        using SSTTreeNodeType = SSTTreeNode<STATE_DIMENSION,CONTROL_DIMENSION>;

        using NodePtr = std::shared_ptr<Node<STATE_DIMENSION>>;
        using TreeNodePtr = std::shared_ptr <TreeNode<STATE_DIMENSION,CONTROL_DIMENSION>>;
        using SSTTreeNodePtr = std::shared_ptr <SSTTreeNode<STATE_DIMENSION,CONTROL_DIMENSION>>;


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
        bool forwardBlossom(const SSTTreeNodePtr & parent,StateType& state,ControlType & control,double& duration){

            if(parent->getTreeId()!=TreeId::forward || !parent->isActive())
                return false;
            bool return_value = false;

            StateType temp_state;
            double temp_duration;
            auto heuristic_value = this->_dynamic_system->getHeuristic(parent->getState(),this->_goal->getState());

            ///propagate M times and select a best node with least estimated solution cost
            for(int i=0;i<PlannerConfig::blossomM;i++) {
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,
                                                                   steps,temp_state,temp_duration)){
                    if(this->_dynamic_system->getHeuristic(temp_state,this->_goal->getState())<heuristic_value){
                        return_value = true;
                        state=temp_state;
                        control=temp_control;
                        duration=temp_duration;
                        heuristic_value = this->_dynamic_system->getHeuristic(temp_state,this->_goal->getState())<heuristic_value;
                    }
                }
            }
            return return_value;
        }

        /***
         * backward blossom
         * @param parent the node to be explored
         * @param state store the temp state
         * @param control store the temp control
         * @param duration store temp duration
         * @return true if blossom process success
         */
        bool backwardBlossom(const SSTTreeNodePtr& parent,StateType & state,ControlType & control,double& duration){
            if(parent->getTreeId()!=TreeId::backward || !parent->isActive())
                return false;
            bool return_value = false;
            StateType temp_state;
            double temp_duration;
            auto heuristic_value = this->_dynamic_system->getHeuristic(parent->getState(),this->_root->getState());

            for(int i=0;i<PlannerConfig::blossomM;i++) {
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->backwardPropagateBySteps(parent->getState(),temp_control,
                                                                   steps,temp_state,temp_duration)){

                    if(this->_dynamic_system->getHeuristic(temp_state,this->_root->getState())<heuristic_value){
                        return_value = true;
                        state=temp_state;
                        control=temp_control;
                        duration=temp_duration;
                        heuristic_value = this->_dynamic_system->getHeuristic(temp_state,this->_root->getState())<heuristic_value;
                    }
                }
            }
            return return_value;
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

            StateType state;
            ControlType control;
            double duration;
            ///cast the node to sst type
            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNodeType>(parent);

            ///perform blossom, until it fails
            while (forwardBlossom(sst_parent,state,control,duration)){
                if(sst_parent== nullptr)return;
                auto new_node = this->addToTree(TreeId::forward,sst_parent,state,control,duration);
                this->checkConnection(new_node);
                if(PlannerConfig::show_node && new_node!= nullptr){
                    std::thread t([this,state](){
                        this->notifyNodeAdded(state,TreeId::forward);
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

            StateType state;
            ControlType control;
            double duration;
            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNodeType>(parent);
            while (backwardBlossom(sst_parent,state,control,duration)){
                auto new_node = this->addToTree(TreeId::backward,sst_parent,state,control,duration);
                this->checkConnection(new_node);
                if(PlannerConfig::show_node && new_node!= nullptr){
                    std::thread t([this,state](){
                        this->notifyNodeAdded(state,TreeId::backward);
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
