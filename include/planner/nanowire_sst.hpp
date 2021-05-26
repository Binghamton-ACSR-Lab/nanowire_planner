//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_SST_HPP
#define NANOWIREPLANNER_NANOWIRE_SST_HPP
#include "nanowire_planner.hpp"
#include "mutex"
#include <unordered_map>
namespace acsr{

    /***
     * hash function for key = IVector
     */
    struct GridHash
    {
        std::size_t operator()(const IVector & k) const
        {
            auto v = std::hash<int>()(k(0));
            for(auto i=1;i<k.size();++i){
                v = (v<< 4) ^ std::hash<int>()(k(i));
            }

            return v;
        }
    };

    class SST : public Planner {
    protected:
        /***
         * override function
         * @param init_state
         * @param control
         * @param step_length
         * @param steps
         * @param result_state
         * @param duration
         * @return
         */
        virtual bool forwardPropagate(const Eigen::VectorXd& init_state, const Eigen::VectorXd& control, int steps,Eigen::VectorXd& result_state,double& duration){
            return this->_dynamic_system->forwardPropagateBySteps(init_state,control,steps,result_state,duration);
        }

        /***
         * override function
         * @param init_state
         * @param control
         * @param step_length
         * @param steps
         * @param result_state
         * @param duration
         * @return
         */
        virtual bool backwardPropagate(const Eigen::VectorXd& init_state, const Eigen::VectorXd& control, int steps,Eigen::VectorXd& result_state,double& duration){
            return this->_dynamic_system->backwardPropagateBySteps(init_state,control,steps,result_state,duration);
        }

        /***
         * add a node to tree
         * @param tree_id tree id
         * @param parent the parent node
         * @param state the state of the node
         * @param control the contrl of the edge
         * @param duration the duration of the edge
         */
        virtual SSTTreeNodePtr addToTree(TreeId tree_id,
                                     TreeNodePtr parent,
                                     const Eigen::VectorXd& state,
                                     const Eigen::VectorXd& control,
                                     double duration){
            auto target_node = tree_id==TreeId::forward?_goal:_root;
            if(_best_cost <= parent->getCost() + duration + _dynamic_system->getHeuristic(state,target_node->getState()))
                return nullptr;

            auto& map = tree_id==TreeId::forward?forward_prox_map:backward_prox_map;
            auto iv = _dynamic_system->getGrid(state);
            SSTTreeNodePtr previous_node = nullptr;
            if(map.find(iv)!=map.end() && !map[iv].expired())
                previous_node = map[iv].lock();
            if(previous_node
               && previous_node->getCost() < parent->getCost() + duration)
                return nullptr;

            ///create a new tree node
            auto new_node = std::make_shared<SSTTreeNode>(tree_id,state);
            new_node->setEdge({control,duration});
            new_node->setParent(parent);
            new_node->setCost(parent->getCost() + duration);
            ///set parent's child
            parent->addChild(new_node);

            /// the original witness has a monitor node but that node has greater cost than the node we are working
            if(previous_node){
                previous_node->setActive(false);
                removePointFromKdTree(previous_node);
                while( previous_node->getChildren().empty() && !previous_node->isActive() && !this->isInSolutionPath(previous_node)){
                    auto next = previous_node->getParent();
                    removeLeaf(previous_node);
                    previous_node = std::static_pointer_cast<SSTTreeNode>(next);
                }
            }

            ///set new node as the current active point
            map[iv] = new_node;
            new_node->setActive(true);
            addPointToKdTree(new_node);
            return new_node;
        }

        /***
         * check the distance of a new node to another tree
         * @param node
         */
        void checkConnection(const TreeNodePtr& node){
            if(node == nullptr)return;
            TreeId another_treeid = (node->getTreeId()==TreeId::forward?TreeId::backward:TreeId::forward);
            auto nearest_vec_node = getNearNodeByRadiusAndNearest(node->getState(),another_treeid,PlannerConfig::optimization_distance);
            ///there're nodes within optimization_distance
            if(!nearest_vec_node.first.empty()){
                for(auto& n : nearest_vec_node.first){
                    auto pair_node = std::static_pointer_cast<TreeNode>(n);
                    if(pair_node->getCost()+node->getCost() >= _best_cost)continue;
                    ///distance within goal radius, trying to update solution
                    if(this->_dynamic_system->distance(pair_node->getState(),node->getState())<=this->_goal_radius){
                        if(node->getTreeId() == TreeId::forward) {
                            this->_best_goal.first = node;
                            this->_best_goal.second = std::static_pointer_cast<TreeNode>(n);
                        }else{
                            this->_best_goal.first = std::static_pointer_cast<TreeNode>(n);
                            this->_best_goal.second = node;
                        }
                        updateBestCost();
                        notifySolutionUpdate();
                        branchBound(_root);
                        branchBound(_goal);
                    }else{
                        auto d = pair_node->getCost()+node->getCost()+_dynamic_system->getHeuristic(pair_node->getState(),node->getState());
                        std::scoped_lock<std::mutex> lock(optimize_set_mutex);
                        if(node->getTreeId()==TreeId::forward) {
                            optimize_set[d] = std::make_pair(node,pair_node);
                        }else{
                            optimize_set[d] = std::make_pair(pair_node,node);
                        }
                    }
                }

            }
        }

        /***
         * add node to kdtree
         * @param node
         */
        virtual void addPointToKdTree(TreeNodePtr node){
            if(node->getTreeId()==TreeId::forward){
                std::scoped_lock<std::mutex> lock1(forward_tree_mutex);
                forward_tree.insert({node->getState(),node});
                this->_number_of_nodes.first+=1;
            }
            else if(node->getTreeId()==TreeId::backward){
                std::scoped_lock<std::mutex> lock1(backward_tree_mutex);
                backward_tree.insert({node->getState(),node});
                this->_number_of_nodes.second+=1;
            }
        }

        /***
         * remove node from tree
         * @param node
         */
        virtual void removePointFromKdTree(TreeNodePtr node){
            if(node== nullptr )
                return;

            if(node->getTreeId()==TreeId::forward){
                std::scoped_lock<std::mutex> lock1(forward_tree_mutex);
                if(forward_tree.find(node->getState())!=forward_tree.end()) {
                    this->forward_tree.erase(node->getState());
                    this->_number_of_nodes.first -= 1;
                }
            }
            else if(node->getTreeId()==TreeId::backward){
                std::scoped_lock<std::mutex> lock1(backward_tree_mutex);
                if(backward_tree.find(node->getState())!=backward_tree.end()) {
                    backward_tree.erase(node->getState());
                    this->_number_of_nodes.second -= 1;
                }
            }
        }

        /***
         * get nearest n nodes
         * @param state
         * @param tree_id
         * @param count
         * @return
         */
        virtual std::vector<NodePtr> getNearNodeByCount(const Eigen::VectorXd& state,TreeId tree_id,int count){
            std::vector<NodePtr> temp_vec;
            if(tree_id == TreeId::forward){
                std::unique_lock<std::mutex> lock(forward_tree_mutex);
                //temp_vec =  _dynamic_system->getNearNodeByCount(state,forward_tree,count);
                auto it = spatial::quadrance_neighbor_begin(forward_tree, state);
                auto max_count = std::min(int(forward_tree.size()), count);
                temp_vec.resize(max_count);
                for (auto i = 0; i < max_count; ++i) {
                    temp_vec[i] = it->second;
                    ++it;
                }
            }else if(tree_id == TreeId::backward){
                std::unique_lock<std::mutex> lock(backward_tree_mutex);
                auto it = spatial::quadrance_neighbor_begin(backward_tree, state);
                auto max_count = std::min(int(backward_tree.size()), count);
                temp_vec.resize(max_count);
                for (auto i = 0; i < max_count; ++i) {
                    temp_vec[i] = it->second;
                    ++it;
                }
            }
            return temp_vec;
        }

        /***
         * get node within a radius arount state
         * @param state
         * @param tree_id
         * @param radius
         * @return first: the vector containing all nodes within radius around state, second: the nearest node, no matter whether it's in radius
         */
        virtual std::pair<std::vector<NodePtr>,NodePtr>
        getNearNodeByRadiusAndNearest(const Eigen::VectorXd& state,TreeId tree_id,double radius){
            std::pair<std::vector<NodePtr>,NodePtr> temp_node_vec;
            if(tree_id == TreeId::forward){
                std::unique_lock<std::mutex> lock(forward_tree_mutex);
                temp_node_vec = _dynamic_system->getNearNodeByRadiusAndNearest(state,forward_tree,radius);
            }else if(tree_id == TreeId::backward){
                std::unique_lock<std::mutex> lock(backward_tree_mutex);
                temp_node_vec = _dynamic_system->getNearNodeByRadiusAndNearest(state,backward_tree,radius);
            }
            return temp_node_vec;
        }

        /***
         * select a pair node for optimizing connecting
         * @param node
         * @param pair_node
         * @return
         */
        virtual double chooseOtherNearestForOptimization(TreeNodePtr node,TreeNodePtr & pair_node){
            double min_value=-1.0;
            if(node== nullptr )return -1;
            TreeId other_tree_id = node->getTreeId()==TreeId::backward?TreeId::forward:TreeId::backward;
            auto nearest = getNearNodeByRadiusAndNearest(node->getState(),other_tree_id,PlannerConfig::optimization_distance);
            if(nearest.first.empty()){
                pair_node = nullptr;
                return min_value;
            }

            pair_node = nullptr;
            min_value = std::numeric_limits<double>::max();

            for(auto temp_pair_node:nearest.first){
                auto temp_tree_node = std::dynamic_pointer_cast<SSTTreeNode>(temp_pair_node);
                if(!temp_tree_node->isActive())
                    continue;
                auto temp_cost = this->_dynamic_system->getHeuristic(temp_tree_node->getState(),node->getState()) + temp_tree_node->getCost();
                if( temp_cost < min_value){
                    min_value=temp_cost;
                    pair_node = temp_tree_node;
                }
            }
            return pair_node == nullptr?-1:min_value + node->getCost()+_dynamic_system->getHeuristic(node->getState(),pair_node->getState());
        }

        /***
         * remove leaf node
         * @param node
         */
        virtual void removeLeaf(TreeNodePtr node){
            if(node->getParent()){
                node->getParent()->removeChild(node);
            }
        }

        /***
         * branch trim
         * @param node
         */
        virtual void branchBound(TreeNodePtr node){
            auto children = node->getChildren();

            if(node->getCost() > _best_cost){
                removeBranch(node);
            }else{
                for(auto &n:children){
                    branchBound(n);
                }
            }
        }

        /***
         * remove a branch
         * @param node the branch root going to be removed
         */
        virtual void removeBranch(TreeNodePtr node){
            auto children = node->getChildren();
            if(node->getParent()!=nullptr){
                node->getParent()->removeChild(node);
            }
            auto sst_node = std::static_pointer_cast<SSTTreeNode>(node);
            if(sst_node->isActive()) {
                sst_node->setActive(false);
                removePointFromKdTree(node);
            }
            for(auto& n:children){
                removeBranch(n);
            }
        }

    protected:

        KdTreeType forward_tree;
        KdTreeType backward_tree;
        std::mutex forward_tree_mutex;
        std::mutex backward_tree_mutex;

        std::mutex optimize_set_mutex;
        std::map<double,std::pair<std::weak_ptr<TreeNode>,std::weak_ptr<TreeNode>>> optimize_set;

        SSTTreeNodePtr _root;
        SSTTreeNodePtr _goal;

        std::unordered_map<IVector,std::weak_ptr<SSTTreeNode>,GridHash> forward_prox_map; ///map storing forward active node but in weak_ptr
        std::unordered_map<IVector,std::weak_ptr<SSTTreeNode>,GridHash> backward_prox_map; ///map storing backword active node but in weak_ptr


    public:
        SST() = delete;

        /***
         * constructor
         * @param dynamic_system
         */
        explicit SST(const std::shared_ptr<NanowireSystem>& dynamic_system):
            Planner(dynamic_system),
            forward_tree(this->_dynamic_system->getStateDimension()),
            backward_tree(this->_dynamic_system->getStateDimension()){
        }

        /***
         * destructor
         */
        ~SST() override{
            forward_tree.clear();
            backward_tree.clear();
            optimize_set.clear();
            forward_prox_map.clear();
            backward_prox_map.clear();
            _root = nullptr;
            _goal = nullptr;
        }

        /***
         * override setup
         */
        void setup() override{
            _run_flag = true;
            ///initial start and target states,this is necessary since the dimension may not consistent.
            Eigen::VectorXd temp_init_state(this->_dynamic_system->getStateDimension());
            Eigen::VectorXd temp_target_state(this->_dynamic_system->getStateDimension());
            for(auto i=0;i<this->_dynamic_system->getStateDimension();++i){
                temp_init_state[i]=this->_init_state[i];
                temp_target_state[i]=this->_target_state[i];
            }
            _init_state.resize(this->_dynamic_system->getStateDimension());
            _target_state.resize(this->_dynamic_system->getStateDimension());
            _init_state = temp_init_state;
            _target_state = temp_target_state;

            ///init root and goal
            _root = std::make_shared<SSTTreeNode>(TreeId::forward, this->_init_state);
            _root->setEdge(TreeEdge(Eigen::VectorXd(this->_dynamic_system->getControlDimension()),0.0));
            _root->setActive(true);
            addPointToKdTree(_root);
            forward_prox_map[_dynamic_system->getGrid(_init_state)]= _root;

            _goal = std::make_shared<SSTTreeNode>(TreeId::backward, this->_target_state);
            _goal->setEdge(TreeEdge(Eigen::VectorXd(this->_dynamic_system->getControlDimension()),0.0));
            _goal->setActive(true);
            addPointToKdTree(_goal);
            backward_prox_map[_dynamic_system->getGrid(_target_state)]= _goal;
        }

        /***
         * override forward step
         */
        void forwardStep() override{
            auto temp_state = this->_dynamic_system->randomState();
            auto temp_control = this->_dynamic_system->randomControl();
            auto nearest_vec_node = getNearNodeByRadiusAndNearest(temp_state,TreeId::forward,PlannerConfig::sst_delta_near);
            auto parent = nearest_vec_node.second;
            if(!nearest_vec_node.first.empty()) {
                ///select the node with minimum cost
                parent = *std::min_element(nearest_vec_node.first.begin(), nearest_vec_node.first.end(),
                                           [](const NodePtr &node1, const NodePtr &node2) {
                                               return std::static_pointer_cast<TreeNode>(node1)->getCost() < std::static_pointer_cast<TreeNode>(node2)->getCost();
                                           });
            }

            auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
            Eigen::VectorXd result;
            double duration;
            if(forwardPropagate(parent->getState(),temp_control,steps,result,duration)){
                auto new_node = addToTree(TreeId::forward,std::static_pointer_cast<TreeNode>(parent),result,temp_control,duration);
                checkConnection(new_node);
            }
        }

        /***
         * override backward step
         */
        void backwardStep() override{
            if(!_is_bi_tree_planner)
                return;
            auto temp_state = this->_dynamic_system->randomState();
            auto temp_control = this->_dynamic_system->randomControl();
            auto nearest_vec_node = getNearNodeByRadiusAndNearest(temp_state,TreeId::backward,PlannerConfig::sst_delta_near);
            //auto parent = std::static_pointer_cast<SSTTreeNode>(nearest_vec_node.second);
            auto parent = nearest_vec_node.second;
            if(!nearest_vec_node.first.empty()) {
                parent = *std::min_element(nearest_vec_node.first.begin(), nearest_vec_node.first.end(),
                                           [](const NodePtr &node1,const NodePtr &node2) {
                                               return std::static_pointer_cast<TreeNode>(node1)->getCost() < std::static_pointer_cast<TreeNode>(node2)->getCost();
                                           });
            }
            auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
            Eigen::VectorXd result;
            double duration;
            if(backwardPropagate(parent->getState(),temp_control,steps,result,duration)){
                auto new_node = addToTree(TreeId::backward,std::static_pointer_cast<TreeNode>(parent),result,temp_control,duration);
                checkConnection(new_node);
            }
        }

        /***
         * override connecting step
         */
        void connectingStep() override{
            if(!_run_flag)return;
            if(!this->_is_optimized_connect)
                return;
            if(this->optimize_set.empty())
                return;
            TreeNodePtr explore_node = nullptr;
            TreeNodePtr temp_explore_node = nullptr;
            TreeNodePtr target = nullptr;
            TreeNodePtr temp_target = nullptr;

            auto it = this->optimize_set.begin();
            while(it!=this->optimize_set.end()){
                if(!_run_flag)return;
                if(it->second.first.expired() || it->second.second.expired()){
                    it = optimize_set.erase(it);
                    continue;
                }
                explore_node = it->second.first.lock();
                target = it->second.second.lock();
                optimize_set.erase(it);
                break;
            }
            auto h1 = chooseOtherNearestForOptimization(explore_node,temp_target);
            auto h2 = chooseOtherNearestForOptimization(temp_target,temp_explore_node);
            if(h1<0 && h2<0)return;
            double total_cost;
            if(h1<0 && h2>0){
                explore_node=temp_explore_node;
                total_cost=h2;
            }else if(h1>0 && h2<0){
                target=temp_target;
                total_cost=h1;
            }else if(h1>0 && h1<h2){
                target=temp_target;
                total_cost=h1;
            }else{
                explore_node=temp_explore_node;
                total_cost=h2;
            }

            if(explore_node ==nullptr
                || !std::static_pointer_cast<SSTTreeNode>(explore_node)->isActive()
                || target==nullptr
                || !std::static_pointer_cast<SSTTreeNode>(target)->isActive()
                || total_cost > _best_cost){
                return;
            }

            Eigen::MatrixXd vec_state;
            Eigen::MatrixXd vec_control;
            Eigen::VectorXd vec_duration;

            Eigen::VectorXd init_state = explore_node->getState();
            Eigen::VectorXd target_state = target->getState();

            bool optimized = this->_dynamic_system->connect(init_state, target_state,
                                                            vec_state, vec_control,vec_duration);
            if(!_run_flag)return;
            if(explore_node == nullptr || target == nullptr)
                return;

            if(!optimized){
                return;
            }
            if(vec_duration.size()==0)
                return;

            //auto total_duration = std::accumulate(vec_duration.begin(),vec_duration.end(),0.0);
            auto total_duration = vec_duration.sum();
            if(explore_node->getCost() + target->getCost() + total_duration < _best_cost){

                this->_best_goal.first = explore_node;
                this->_best_goal.second = target;

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
                updateBestCost();
                branchBound(this->_root);
                branchBound(this->_goal);
                notifySolutionUpdate();
            }
        }

        /***
         * given initial state, control and durations to take forward steps
         * @param state start state
         * @param controls control
         * @param duration duration
         * @return final state
         */
        Eigen::VectorXd
        forward(const Eigen::VectorXd &state, const Eigen::VectorXd &controls, const double duration) override {
            auto step_length = _dynamic_system->getStepSize();
            Eigen::VectorXd result;
            double d;
            _dynamic_system->forwardPropagateBySteps(state,controls,duration/step_length,result,d);
            return result;
        }
    };
}


#endif //NANOWIREPLANNER_NANOWIRE_SST_HPP
