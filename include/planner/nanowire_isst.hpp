//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_ISST_HPP
#define NANOWIREPLANNER_NANOWIRE_ISST_HPP
#include "nanowire_sst.hpp"

namespace acsr{

    class iSST: public SST{

    protected:
        ///propagation times in one blossom
        //const unsigned int M = 15;

        ///a map to classify node, {node_ptr, estimated solution cost}
        NodeMapType open_map[2];
        NodeMapType close_map[2];

        /***
         * setup
         */
        void setup() override {
            SST::setup();
            double total_cost = this->_dynamic_system->getHeuristic(this->_init_state,this->_target_state);

            open_map[0].insert({this->_root,total_cost});
            this->_root->setTreeNodeState(in_open_set);
            open_map[1].insert({this->_goal,total_cost});
            this->_goal->setTreeNodeState(in_open_set);

        }

        /***
         * select a node to explore, unlike in SST or RRT, this process is not stochastic. we select a node with least estimated solution cost
         * @param tree_id
         * @param parent
         * @return
         */
        bool searchSelection(TreeId tree_id, TreeNodePtr& parent){
            int index = tree_id==TreeId::forward?0:1;

            ///swap open_map and close_map if no node exits in open_map
            if(open_map[index].empty()){
                std::swap(open_map[index],close_map[index]);
                for(auto pairs:open_map[index]){
                    pairs.first->setTreeNodeState(TreeNodeState::in_open_set);
                }
            }

            ///select the node with minimum estimate solution cost

            auto min_cost_node_it = std::min_element(open_map[index].begin(),open_map[index].end(),
                                                     [](const std::pair<TreeNodePtr,double>& m1, const std::pair<TreeNodePtr,double>& m2){
                                                         return m1.second < m2.second;
                                                     });

            ///select the node and remove from openmap
            parent = min_cost_node_it->first;
            open_map[index].erase(min_cost_node_it);

            ///this might happen if there's a branchUpdate process
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

            if(parent->getTreeId()!=TreeId::forward || parent->getTreeNodeState()==TreeNodeState::not_in_tree)
                return false;
            bool return_value = false;

            Eigen::VectorXd temp_state;
            double temp_duration;
            auto heuristic_value = this->_dynamic_system->getHeuristic(parent->getState(),this->_goal->getState());

            ///propagate M times and select a best node with least estimated solution cost
            for(int i=0;i<Config::blossomM;i++) {
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(Config::min_time_steps,Config::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,Config::integration_step,
                                                                   steps,temp_state,temp_duration)){
                    return_value = true;
                    if(this->_dynamic_system->getHeuristic(temp_state,this->_goal->getState())<heuristic_value){
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
         * reverse blossom
         * @param parent the node to be explored
         * @param state store the temp state
         * @param control store the temp control
         * @param duration store temp duration
         * @return true if blossom process success
         */
        bool reverseBlossom(const SSTTreeNodePtr& parent,Eigen::VectorXd& state,Eigen::VectorXd& control,double& duration){

            if(parent->getTreeId()!=TreeId::reverse || parent->getTreeNodeState()==TreeNodeState::not_in_tree)
                return false;
            bool return_value = false;

            Eigen::VectorXd temp_state;

            double temp_duration;
            auto heuristic_value = this->_dynamic_system->getHeuristic(parent->getState(),this->_root->getState());

            for(int i=0;i<Config::blossomM;i++) {
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(Config::min_time_steps,Config::max_time_steps);
                if (this->_dynamic_system->reversePropagateBySteps(parent->getState(),temp_control,Config::integration_step,
                                                                   steps,temp_state,temp_duration)){
                    return_value = true;
                    if(this->_dynamic_system->getHeuristic(temp_state,this->_root->getState())<heuristic_value){
                        state=temp_state;
                        control=temp_control;
                        duration=temp_duration;
                        heuristic_value = this->_dynamic_system->getHeuristic(temp_state,this->_root->getState())<heuristic_value;
                    }
                }
            }
            return return_value;
        }



        /***
         * remove witness from searching container
         * @param node
         */
        void removeNodeFromSet(TreeNodePtr node) override {
            if(node == nullptr){
                this->optimize_set[0].erase(std::remove(begin(this->optimize_set[0]), end(this->optimize_set[0]), node), end(this->optimize_set[0]));
                this->optimize_set[1].erase(std::remove(begin(this->optimize_set[1]), end(this->optimize_set[1]), node), end(this->optimize_set[1]));

                open_map[0].erase(node);
                open_map[1].erase(node);
                close_map[0].erase(node);
                close_map[1].erase(node);
                return;
            }

            int tree_id = node->getTreeId()==TreeId::forward?0:1;
            if(node->getTreeNodeState() == TreeNodeState::in_optimize_set){
                this->optimize_set[tree_id].erase(std::remove(begin(this->optimize_set[tree_id]), end(this->optimize_set[tree_id]), node), end(this->optimize_set[tree_id]));

            }else if(node->getTreeNodeState() == TreeNodeState::in_open_set){
                open_map[tree_id].erase(node);
            }else if(node->getTreeNodeState() == TreeNodeState::in_close_set){
                close_map[tree_id].erase(node);
            }
            node->setTreeNodeState(TreeNodeState::not_in_set);
        }

    public:
        explicit iSST(std::shared_ptr<NanowireSystem> dynamic_system):SST(dynamic_system){

        }

        iSST()=delete;
        iSST(const iSST&) = delete;

        ~iSST() override{
            this->forward_prox_container.clear();
            this->reverse_prox_container.clear();
            this->forward_tree.clear();
            this->reverse_tree.clear();
            open_map[0].clear();
            open_map[1].clear();
            close_map[0].clear();
            close_map[1].clear();
            this->optimize_set[0].clear();
            this->optimize_set[1].clear();
        }

        /***
         * override forward step
         */
        void forwardStep() override{
            ///select a node to be explored in forward tree
            TreeNodePtr parent;
            searchSelection(TreeId::forward,parent);
            if(parent->getTreeNodeState()==TreeNodeState::not_in_tree)
                return;

            Eigen::VectorXd state;
            Eigen::VectorXd control;
            double duration;
            ///cast the node to sst type
            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNode>(parent);

            ///perform blossom, until it fails
            while (forwardBlossom(sst_parent,state,control,duration)){

                if(sst_parent== nullptr)return;

                ///add parent to close map
                if(sst_parent->getTreeNodeState()==TreeNodeState::in_open_set ||
                   (sst_parent->getTreeNodeState()==TreeNodeState::not_in_set && sst_parent->isActive())){
                    removeNodeFromSet(sst_parent);
                    auto nodes_vec = this->getNearNodeByCount(sst_parent->getState(),TreeId::reverse,5);
                    //std::vector<SSTNodePtr> sstnode_vec(nodes_vec.size());
                    /*
                    std::transform(nodes_vec.begin(), nodes_vec.end(), sstnode_vec.begin(),
                                   [](const NodePtr &node) {
                                       return std::static_pointer_cast<SSTNode<T,Te,Tc>>(node);
                                   });*/

                    auto pair_node = *std::min_element(nodes_vec.begin(),nodes_vec.end(),
                                                       [&](const NodePtr& n1, const NodePtr& n2){
                                                           return std::static_pointer_cast<TreeNode>(n1)->getCost() + this->_dynamic_system->getHeuristic(n1->getState(),sst_parent->getState())
                                                                  < std::static_pointer_cast<TreeNode>(n2)->getCost() + this->_dynamic_system->getHeuristic(n2->getState(),sst_parent->getState());
                                                       });

                    close_map[0].insert({sst_parent,
                                         sst_parent->getCost() + std::static_pointer_cast<TreeNode>(pair_node)->getCost()+this->_dynamic_system->getHeuristic(sst_parent->getState(),pair_node->getState())});

                    sst_parent->setTreeNodeState(TreeNodeState::in_close_set);

                }
                if(state.size()!=this->_dynamic_system->getStateDimension())
                    break;
                ///trying add the new node to tree
                auto new_node = this->addToTree(TreeId::forward,sst_parent,state,control,duration);
                /// check is connectivity to another tree
                this->checkConnection(new_node);

                ///new node not exist or new node is the best_goal
                if(new_node == nullptr || this->_best_goal.first == new_node){
                    break;
                }else {
                    sst_parent = new_node;
                }
            }

            ///add parent to close map
            if(sst_parent->getTreeNodeState()==TreeNodeState::not_in_set && sst_parent->isActive()){

                auto nodes_vec = this->getNearNodeByCount(sst_parent->getState(),TreeId::reverse,5);
                /*std::vector<SSTNodePtr> sstnode_vec(nodes_vec.size());

                std::transform(nodes_vec.begin(), nodes_vec.end(), sstnode_vec.begin(),
                               [](const std::shared_ptr<Node<StateType>> &node) {
                                   return std::static_pointer_cast<SSTNode<StateType,ControlType>>(node);
                               });*/

                auto pair_node = *std::min_element(nodes_vec.begin(),nodes_vec.end(),
                                                   [&](const NodePtr & n1, const NodePtr & n2){
                                                       return std::static_pointer_cast<TreeNode>(n1)->getCost() + this->_dynamic_system->getHeuristic(n1->getState(),sst_parent->getState())
                                                              < std::static_pointer_cast<TreeNode>(n2)->getCost() + this->_dynamic_system->getHeuristic(n2->getState(),sst_parent->getState());
                                                   });
                close_map[0].insert({sst_parent,
                                     sst_parent->getCost() + std::static_pointer_cast<TreeNode>(pair_node)->getCost()+this->_dynamic_system->getHeuristic(sst_parent->getState(),pair_node->getState())});
                sst_parent->setTreeNodeState(TreeNodeState::in_close_set);
            }


        }

        /***
         * override reverse step
         */
        void reverseStep() override{
            if(!this->_is_bi_tree_planner)
                return;
            TreeNodePtr parent;
            searchSelection(TreeId::reverse,parent);
            if(parent->getTreeNodeState()==TreeNodeState::not_in_tree)return;

            Eigen::VectorXd state;
            Eigen::VectorXd control;
            double duration;
            auto sst_parent = std::dynamic_pointer_cast<SSTTreeNode>(parent);
            while (reverseBlossom(sst_parent,state,control,duration)){
                ///add parent to close map
                if(sst_parent->getTreeNodeState()==TreeNodeState::in_open_set ||
                   (sst_parent->getTreeNodeState()==TreeNodeState::not_in_set && sst_parent->isActive())){
                    removeNodeFromSet(sst_parent);
                    auto nodes_vec = this->getNearNodeByCount(sst_parent->getState(),TreeId::forward,5);
                    /*std::vector<std::shared_ptr<SSTNode<StateType,ControlType>>> sstnode_vec(nodes_vec.size());
                    std::transform(nodes_vec.begin(), nodes_vec.end(), sstnode_vec.begin(),
                                   [](const std::shared_ptr<Node<StateType>> &node) {
                                       return std::static_pointer_cast<SSTNode<StateType,ControlType>>(node);
                                   });*/
                    auto pair_node = *std::min_element(nodes_vec.begin(),nodes_vec.end(),
                                                       [&](const NodePtr & n1, const NodePtr & n2){
                                                           return std::static_pointer_cast<TreeNode>(n1)->getCost() + this->_dynamic_system->getHeuristic(n1->getState(),sst_parent->getState())
                                                                  < std::static_pointer_cast<TreeNode>(n2)->getCost() + this->_dynamic_system->getHeuristic(n2->getState(),sst_parent->getState());
                                                       });

                    close_map[1].insert({sst_parent,
                                         sst_parent->getCost() + std::static_pointer_cast<TreeNode>(pair_node)->getCost()+this->_dynamic_system->getHeuristic(sst_parent->getState(),pair_node->getState())});

                    sst_parent->setTreeNodeState(TreeNodeState::in_close_set);

                }
                if(state.size()!=this->_dynamic_system->getStateDimension())
                    break;
                auto new_node = this->addToTree(TreeId::reverse,sst_parent,state,control,duration);
                this->checkConnection(new_node);
                if(new_node == nullptr || this->_best_goal.second == new_node){
                    break;
                }else
                    sst_parent=new_node;
            }

            if(sst_parent->getTreeNodeState()==TreeNodeState::not_in_set && sst_parent->isActive()){
                auto nodes_vec = this->getNearNodeByCount(sst_parent->getState(),TreeId::forward,5);
                /*std::vector<std::shared_ptr<SSTNode<StateType,ControlType>>> sstnode_vec(nodes_vec.size());
                std::transform(nodes_vec.begin(), nodes_vec.end(), sstnode_vec.begin(),
                               [](const std::shared_ptr<Node<StateType>> &node) {
                                   return std::static_pointer_cast<SSTNode<StateType,ControlType>>(node);
                               });*/

                auto pair_node = *std::min_element(nodes_vec.begin(),nodes_vec.end(),
                                                   [&](const NodePtr & n1, const NodePtr & n2){
                                                       return std::static_pointer_cast<TreeNode>(n1)->getCost() + this->_dynamic_system->getHeuristic(n1->getState(),sst_parent->getState())
                                                              < std::static_pointer_cast<TreeNode>(n2)->getCost() + this->_dynamic_system->getHeuristic(n2->getState(),sst_parent->getState());
                                                   });

                close_map[1].insert({sst_parent,
                                     sst_parent->getCost() + std::static_pointer_cast<TreeNode>(pair_node)->getCost()+this->_dynamic_system->getHeuristic(sst_parent->getState(),pair_node->getState())});

                sst_parent->setTreeNodeState(TreeNodeState::in_close_set);

            }

        }

        /***
         * override connecting step
         */
        void connectingStep() override{
            if(!this->_is_optimized_connect)
                return;

            if(this->optimize_set[0].empty() && this->optimize_set[1].empty())
                return;
            TreeNodePtr explore_node = nullptr;
            TreeNodePtr target = nullptr;
            TreeNodePtr temp_target = nullptr;
            //mutex_for_set.lock();
            auto index = randomInteger(0,1);
            if(this->optimize_set[index].empty())
                index = 1-index;

            double total_cost = std::numeric_limits<double>::max();

            {
                std::scoped_lock<std::mutex> lock(optimize_set_mutex[index]);
                auto it = this->optimize_set[index].begin();
                while (it != this->optimize_set[index].end()) {
                    if (*it == nullptr || (*it)->getTreeNodeState() == TreeNodeState::not_in_tree) {
                        auto node = *it;
                        it = this->optimize_set[index].erase(it);
                        this->removePointFromContainer(node);
                        continue;
                    }
                    auto temp_cost = chooseOtherNearestForOptimization(*it, temp_target);

                    if (temp_target == nullptr || temp_cost > this->getMaxCost()) {
                        auto node = *it;
                        node->setTreeNodeState(in_close_set);
                        it = this->optimize_set[index].erase(it);
                        close_map[index].insert({node, std::numeric_limits<double>::max()});
                        continue;
                    }

                    if (temp_cost < total_cost) {
                        total_cost = temp_cost;
                        explore_node = *it;
                        target = temp_target;
                    }
                    ++it;
                }
            }

            if(explore_node ==nullptr || total_cost > this->getMaxCost())
            {
                return;
            }

            Eigen::MatrixXd vec_state;
            Eigen::MatrixXd vec_control;
            Eigen::VectorXd vec_duration;

            Eigen::VectorXd init_state;
            Eigen::VectorXd target_state;
            if(explore_node == nullptr || target == nullptr)
                return;

            if(explore_node->getTreeId()==TreeId::forward){
                init_state = explore_node->getState();
                target_state = target->getState();
            }else{
                init_state = target->getState();
                target_state = explore_node->getState();
            }

            bool optimized = this->_dynamic_system->connect(init_state, target_state,
                                                            Config::integration_step,
                                                            vec_state, vec_control,vec_duration);
            if(explore_node == nullptr || target == nullptr)
                return;

            removeNodeFromSet(explore_node);
            if(!optimized){
                return;
            }
            if(vec_duration.size()==0)
                return;

            //auto total_duration = std::accumulate(vec_duration.begin(),vec_duration.end(),0.0);
            auto total_duration = vec_duration.sum();
            if(explore_node->getCost() + target->getCost() + total_duration < this->getMaxCost()){
                if(explore_node->getTreeId() == TreeId::forward) {
                    this->_best_goal.first = explore_node;
                    this->_best_goal.second = target;
                }else{
                    this->_best_goal.first = target;
                    this->_best_goal.second = explore_node;
                }

                std::vector<Eigen::VectorXd> connect_states;
                std::vector<Eigen::VectorXd> connect_controls;
                std::vector<double> connect_durations;
                for(auto i=0;i<vec_state.rows()-1;++i){
                    connect_states.push_back(vec_state.row(i));
                }
                for(auto i=0;i<vec_control.rows();++i){
                    connect_controls.push_back(vec_control.row(i));
                }
                for(auto i=0;i<vec_duration.size();++i){
                    connect_durations.push_back(vec_duration(i));
                }
                _planner_connection = std::make_shared<PlannerConnection>(this->_best_goal,connect_states,connect_controls,connect_durations);
                branchBound(this->_root);
                branchBound(this->_goal);
                notifySolutionUpdate();
            }
        }
    };
}



#endif //NANOWIREPLANNER_NANOWIRE_ISST_HPP
