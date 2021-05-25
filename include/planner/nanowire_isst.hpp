//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_ISST_HPP
#define NANOWIREPLANNER_NANOWIRE_ISST_HPP
#include "nanowire_sst.hpp"

namespace acsr{

    class iSST: public SST{

    protected:

        /***
         * setup
         */
        void setup() override {
            SST::setup();
        }

        /***
         * select a node to explore, unlike in SST or RRT, this process is not stochastic. we select a node with least estimated solution cost
         * @param tree_id
         * @param parent
         * @return
         */
        bool searchSelection(TreeId tree_id, TreeNodePtr& parent){
            auto point = _dynamic_system->randomState();
            auto near = getNearNodeByCount(point,tree_id,10);
            auto it = std::min_element(near.begin(), near.end(),
                                       [](const NodePtr &node1, const NodePtr &node2) {
                                           return std::static_pointer_cast<TreeNode>(node1)->getCost() < std::static_pointer_cast<TreeNode>(node2)->getCost();
                                       });
            parent = std::static_pointer_cast<TreeNode>(*it);
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
        bool forwardBlossom(const SSTTreeNodePtr & parent,Eigen::VectorXd& state,Eigen::VectorXd& control,double& duration){

            if(parent->getTreeId()!=TreeId::forward || !parent->isActive())
                return false;
            bool return_value = false;

            Eigen::VectorXd temp_state;
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
        bool backwardBlossom(const SSTTreeNodePtr& parent,Eigen::VectorXd& state,Eigen::VectorXd& control,double& duration){
            if(parent->getTreeId()!=TreeId::backward || !parent->isActive())
                return false;
            bool return_value = false;
            Eigen::VectorXd temp_state;
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
        explicit iSST(std::shared_ptr<NanowireSystem> dynamic_system):SST(dynamic_system){
        }

        iSST()=delete;
        iSST(const iSST&) = delete;

        ~iSST() override{
            this->forward_tree.clear();
            this->backward_tree.clear();
            this->optimize_set.clear();
        }

        /***
         * override forward step
         */
        void forwardStep() override{
            ///select a node to be explored in forward tree
            TreeNodePtr parent;
            searchSelection(TreeId::forward,parent);
            if(!std::static_pointer_cast<SSTTreeNode>(parent)->isActive())
                return;

            Eigen::VectorXd state;
            Eigen::VectorXd control;
            double duration;
            ///cast the node to sst type
            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNode>(parent);

            ///perform blossom, until it fails
            while (forwardBlossom(sst_parent,state,control,duration)){
                if(sst_parent== nullptr)return;
                auto new_node = this->addToTree(TreeId::forward,sst_parent,state,control,duration);
                this->checkConnection(new_node);
                if(PlannerConfig::show_node && new_node!= nullptr){
                    std::thread t([this,state](){
                        notifyNodeAdded(state,TreeId::forward);
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
            if(!std::static_pointer_cast<SSTTreeNode>(parent)->isActive())return;

            Eigen::VectorXd state;
            Eigen::VectorXd control;
            double duration;
            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNode>(parent);
            while (backwardBlossom(sst_parent,state,control,duration)){
                auto new_node = this->addToTree(TreeId::backward,sst_parent,state,control,duration);
                this->checkConnection(new_node);
                if(PlannerConfig::show_node && new_node!= nullptr){
                    std::thread t([this,state](){
                        notifyNodeAdded(state,TreeId::backward);
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
