//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_REF_ISST_HPP
#define NANOWIREPLANNER_NANOWIRE_REF_ISST_HPP

#include <boost/filesystem.hpp>
#include "nanowire_sst.hpp"
#include "global_path.hpp"
#include <math.h>
namespace acsr{

    typedef struct{
        Eigen::VectorXd state;
        Eigen::VectorXd control;
        double duration;
    } PropagateParameters;

    class RefSST: public SST{

    protected:
        ///propagation times in one blossom
        //const unsigned int M = 15;

        ///a map to classify node, {node_ptr, estimated solution cost}
        //NodeMapType open_map[2];
        //NodeMapType close_map[2];

        /***
         * setup
         */
        void setup(const std::shared_ptr<NanowireConfig>& nanowire_config) override {
            SST::setup(nanowire_config);
            //double total_cost = this->_dynamic_system->getHeuristic(this->_init_state,this->_target_state);

            //open_map[0].insert({this->_root,total_cost});
            //this->_root->setTreeNodeState(in_open_set);
            //open_map[1].insert({this->_goal,total_cost});
            //this->_goal->setTreeNodeState(in_open_set);
            GlobalPath global_path;
            //auto nanowire_config = std::make_shared<NanowireConfig>();
            //nanowire_config->readFile("config/nanowire.cfg");
            global_path.init(_init_state,_target_state,nanowire_config);
            global_path.getStartTargetElectordes();
            VariablesGrid states;
            global_path.generateBestReferenceTrajectory(states);

            reference_path = std::make_shared<ReferencePath>();
            reference_path->readFile("reference_path.txt");

            //std::cout<<reference_path->getStates()<<std::endl;

            //_dynamic_system->setReferencePath(reference_path);

            std::vector<double> length(_dynamic_system->getRobotCount(),0.0);
            for(auto j=0;j<states.getNumPoints()-1;++j){
                for(auto i=0;i<_dynamic_system->getRobotCount();++i){
                    auto temp_length=(states(j,2*i)-states(j+1,2*i))*(states(j,2*i)-states(j+1,2*i))+
                            (states(j,2*i+1)-states(j+1,2*i+1))*(states(j,2*i+1)-states(j+1,2*i+1));
                    length[i]+=temp_length;
                }
            }
            std::vector<std::pair<int,double>> length_with_index;
            for(auto i=0;i<length.size();++i)
                length_with_index.push_back({i,length[i]});
            std::sort(length_with_index.begin(),length_with_index.end(),[](const auto& p1,const auto& p2){
                return p1.second>p2.second;
            });
            //_dominant_index = {length_with_index[0].first,length_with_index[1].first};
            //_dominant_index = {length_with_index[0].first};
            auto v = std::min(Config::dominant_path_count,_dynamic_system->getRobotCount());
            for(auto i=0;i<v;++i)
                _dominant_index.push_back(length_with_index[i].first);
        }

        /***
         * select a node to explore, unlike in SST or RRT, this process is not stochastic. we select a node with least estimated solution cost
         * @param tree_id
         * @param parent
         * @return
         */
        bool searchSelection(TreeId tree_id, TreeNodePtr& parent){

            //auto& other_tree = tree_id==TreeId::forward?reverse_tree:forward_tree;
            //auto other_tree_id = tree_id==TreeId::forward?TreeId::reverse:TreeId::forward;
            std::uniform_real_distribution<double> distribution(0.0,1.0);
            auto p = distribution(_random_engine);
            parent = nullptr;
            if(p<0.5){
                auto point = getRandomReferencePoint();
                auto near = getNearNodeByRadiusAndNearest(point,tree_id,50e-6);
                parent = std::static_pointer_cast<TreeNode>(near.second);
                if(!near.first.empty()) {
                    auto it = std::min_element(near.first.begin(), near.first.end(),
                                               [](const NodePtr &node1,const NodePtr &node2) {
                                                   return std::static_pointer_cast<TreeNode>(node1)->getCost() < std::static_pointer_cast<TreeNode>(node2)->getCost();
                                               });
                    parent = std::static_pointer_cast<TreeNode>(*it);
                }
            }else{
                auto point = _dynamic_system->randomState();
                auto near = getNearNodeByCount(point,tree_id,10);
                auto it = std::min_element(near.begin(), near.end(),
                                           [](const NodePtr &node1, const NodePtr &node2) {
                                               return std::static_pointer_cast<TreeNode>(node1)->getCost() < std::static_pointer_cast<TreeNode>(node2)->getCost();
                                           });
                parent = std::static_pointer_cast<TreeNode>(*it);
            }
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
        bool forwardBlossom(const TreeNodePtr & parent,std::vector<PropagateParameters>& params){

            if(parent->getTreeId()!=TreeId::forward || parent->getTreeNodeState()==TreeNodeState::not_in_tree)
                return false;
            bool return_value = false;
            std::map<double,PropagateParameters> quality_map;
            Eigen::VectorXd temp_state;
            double temp_duration;
            //auto heuristic_value = this->_dynamic_system->getHeuristic(parent->getState(),this->_goal->getState());

            ///propagate M times and select a best node with least estimated solution cost
            for(int i=0;i<Config::blossomM;i++) {
                if(!_run_flag)return false;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(Config::min_time_steps,Config::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,Config::integration_step,
                                                                   steps,temp_state,temp_duration)){
                    return_value = true;
                    quality_map[getQuality(parent->getCost() + temp_duration,temp_state,TreeId::forward)] =
                            PropagateParameters {temp_state,temp_control,temp_duration};
                }
            }
            if(return_value){
                int i = 0;
                for(auto it = quality_map.rbegin();i<Config::blossomN && it!=quality_map.rend();++it,++i){
                    params.push_back(it->second);
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
        bool reverseBlossom(const TreeNodePtr& parent,std::vector<PropagateParameters>& params){

            if(parent->getTreeId()!=TreeId::reverse || parent->getTreeNodeState()==TreeNodeState::not_in_tree)
                return false;
            bool return_value = false;

            Eigen::VectorXd temp_state;
            std::map<double,PropagateParameters> quality_map;
            double temp_duration;
            //auto heuristic_value = this->_dynamic_system->getHeuristic(parent->getState(),this->_root->getState());

            for(int i=0;i<Config::blossomM;i++) {
                if(!_run_flag)return false;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(Config::min_time_steps,Config::max_time_steps);
                if (this->_dynamic_system->reversePropagateBySteps(parent->getState(),temp_control,Config::integration_step,
                                                                   steps,temp_state,temp_duration)){
                    return_value = true;
                    quality_map[getQuality(parent->getCost() + temp_duration,temp_state,TreeId::reverse)] =
                            PropagateParameters {temp_state,temp_control,temp_duration};
                }
            }
            if(return_value){
                int i = 0;
                for(auto it = quality_map.rbegin();i<Config::blossomN && it!=quality_map.rend();++it,++i){
                    params.push_back(it->second);
                }
            }
            return return_value;
        }



        /***
         * remove witness from searching container
         * @param node

        void removeNodeFromSet(TreeNodePtr node) override {

            int tree_id = node->getTreeId()==TreeId::forward?0:1;
            if(node->getTreeNodeState() == TreeNodeState::in_optimize_set){
                this->optimize_set[tree_id].erase(std::remove(begin(this->optimize_set[tree_id]), end(this->optimize_set[tree_id]), node), end(this->optimize_set[tree_id]));

            }
            node->setTreeNodeState(TreeNodeState::not_in_set);
        }*/

        Eigen::VectorXd getRandomReferencePoint(){
            //std::default_random_engine engine(_random_engine);
            std::uniform_int_distribution<int> distribution(0,reference_path->getStates().getNumPoints()-1);
            auto index = distribution(_random_engine);
            return reference_path->getStates().getVector(index);
        }

        double getQuality(double time,  const Eigen::VectorXd &state, const TreeId treeid){
            double value;
            auto max_time = reference_path->getMaxTime();
            Eigen::VectorXd reference_state;
            if(treeid==TreeId::forward)
                reference_state= reference_path->getState(time);
            else if(treeid==TreeId::reverse){
                if(time<max_time)
                    reference_state = reference_path->getState(max_time-time);
                else
                    reference_state= reference_path->getState(0);
            }
            Eigen::VectorXd dominant_ref(2*_dominant_index.size());
            Eigen::VectorXd dominant_state(2*_dominant_index.size());

            Eigen::VectorXd non_dominant_ref(state.size()-2*_dominant_index.size());
            Eigen::VectorXd non_dominant_state(state.size()-2*_dominant_index.size());

            int j=0,k=0;
            for(auto i=0;i<_dynamic_system->getRobotCount();++i){
                if(std::find(_dominant_index.begin(),_dominant_index.end(),i)!=_dominant_index.end()){
                    dominant_ref(2*j) = reference_state(2*i);
                    dominant_ref(2*j+1) = reference_state(2*i+1);
                    dominant_state(2*j) = state(2*i);
                    dominant_state(2*j+1) = state(2*i+1);
                    j++;
                }else{
                    non_dominant_ref(2*k) = reference_state(2*i);
                    non_dominant_ref(2*k+1) = reference_state(2*i+1);
                    non_dominant_state(2*k) = state(2*i);
                    non_dominant_state(2*k+1) = state(2*i+1);
                    //non_dominant_target(2*k) = target(2*i);
                    //non_dominant_target(2*k+1) = target(2*i+1);
                    k++;
                }
            }


            double mul = 100.0;
            auto c = M_E/10.0;
            if(time <= max_time){
                //auto p = max_time/time;
                //auto t = (1.0-p)*(1-p);
                auto q = (time/max_time-1.0)*Config::quality_factor;
                //value = 1.0/((dominant_ref-dominant_state).norm()*t) + 0.1/((non_dominant_ref-non_dominant_state).norm()*t);
                //value = std::pow(a,time-max_time)/(dominant_ref-dominant_state).norm() + 0.1*std::pow(a,time-max_time)/(non_dominant_ref-non_dominant_state).norm();
                //value = 1.0/(reference_state-state).norm();
                //value = std::pow(a,time-max_time)/(dominant_ref-dominant_state).norm() + mul*(non_dominant_ref-non_dominant_state).norm();
                //value = std::pow(c,q)/(dominant_ref-dominant_state).norm() + 0.5*std::pow(c,q)/(non_dominant_ref-non_dominant_state).norm();
                auto z = std::exp(q);
                value = z/(dominant_ref-dominant_state).norm();//+0.001 * z/(non_dominant_ref-non_dominant_state).norm();
            }else {
                //auto p = time/max_time;
                //auto t = (p-1.0)*(p-1)*(p-1);
                auto q = Config::quality_decrease_factor*(max_time/time-1.0)*Config::quality_factor;
                //value = 1.0/((dominant_ref-dominant_state).norm()*t) + 0.1/((non_dominant_ref-non_dominant_state).norm()*t);
                //value = std::pow(b,max_time-time)/(dominant_ref-dominant_state).norm() +0.1*std::pow(b,max_time-time)/(non_dominant_ref-non_dominant_state).norm();;
                //value = 1.0/(reference_state-state).norm();
                //value = std::pow(b,max_time-time)/(dominant_ref-dominant_state).norm() +mul*(non_dominant_ref-non_dominant_state).norm();
                //value = std::pow(c,q)/(dominant_ref-dominant_state).norm() +0.5*std::pow(c,q)/(non_dominant_ref-non_dominant_state).norm();
                auto z = std::exp(q);
                value = z/(dominant_ref-dominant_state).norm();//+0.001 * z/(non_dominant_ref-non_dominant_state).norm();
            }
            return value;
        }



    public:
        explicit RefSST(std::shared_ptr<NanowireSystem> dynamic_system):SST(dynamic_system){

        }

        RefSST()=delete;
        RefSST(const RefSST&) = delete;

        ~RefSST() override{
            //this->forward_prox_container.clear();
            //this->reverse_prox_container.clear();
            this->forward_tree.clear();
            this->reverse_tree.clear();
            //open_map[0].clear();
            //open_map[1].clear();
            //close_map[0].clear();
            //close_map[1].clear();
            optimize_set.clear();
        }

        /***
         * override forward step
         */
        void forwardStep() override{
            ///select a node to be explored in forward tree
            TreeNodePtr parent= nullptr;
            if(!searchSelection(TreeId::forward,parent))return;
            if(parent->getTreeNodeState()==TreeNodeState::not_in_tree)
                return;
            std::vector<PropagateParameters> params;
            if (forwardBlossom(parent,params)){
                if(parent==nullptr)
                    return;
                for(auto param:params){
                    auto new_node = addToTree(TreeId::forward,parent,param.state,param.control,param.duration);
                    if(Config::show_node && new_node!= nullptr){
                        auto state = new_node->getState();
                        std::thread t([this,state](){
                            Eigen::VectorXd s(2*_dominant_index.size());
                            for(auto i=0;i<_dominant_index.size();++i){
                                s[2*i] = state[2*_dominant_index[i]];
                                s[2*i+1] = state[2*_dominant_index[i]+1];
                            }
                            notifyNodeAdded(state,TreeId::forward);
                        });
                        t.detach();
                    }
                    checkConnection(new_node);
                }
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

            std::vector<PropagateParameters> params;
            if (reverseBlossom(parent,params)){
                ///add parent to close map
                if(parent==nullptr || parent->getTreeNodeState()==TreeNodeState::not_in_tree)
                    return;
                for(auto& param:params){
                    auto new_node = addToTree(TreeId::reverse,parent,param.state,param.control,param.duration);
                    if(Config::show_node && new_node!= nullptr){
                        auto state = new_node->getState();
                        std::thread t([this,state](){
                            Eigen::VectorXd s(2*_dominant_index.size());
                            for(auto i=0;i<_dominant_index.size();++i){
                                s[2*i] = state[2*_dominant_index[i]];
                                s[2*i+1] = state[2*_dominant_index[i]+1];
                            }
                            notifyNodeAdded(state,TreeId::reverse);
                        });
                        t.detach();

                    }
                    this->checkConnection(new_node);
                }
            }
        }

        /***
         * override connecting step
         */
        /*void connectingStep() override{
            if(!_run_flag)return;
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
            auto it = this->optimize_set[index].begin();
            while(it!=this->optimize_set[index].end()){
                if(!_run_flag)return;
                TreeNodePtr n = nullptr;
                if((*it).expired()){
                    it = optimize_set[index].erase(it);
                    continue;
                }
                n=it->lock();
                if(n == nullptr || n->getTreeNodeState()==TreeNodeState::not_in_tree) {
                    it = optimize_set[index].erase(it);
                    continue;
                }
                auto temp_cost = chooseOtherNearestForOptimization(n,temp_target);
                if(temp_target==nullptr || temp_cost>this->getMaxCost()){
                    //n->setTreeNodeState(TreeNodeState::not_in_set);
                    it = this->optimize_set[index].erase(it);
                    continue;
                }

                if(temp_cost < total_cost){
                    total_cost =temp_cost;
                    explore_node = n;
                    target=temp_target;
                }
                ++it;
            }

            if(explore_node ==nullptr || explore_node->getTreeNodeState()==TreeNodeState::not_in_tree || total_cost > this->getMaxCost())
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
            if(!_run_flag)return;
            if(explore_node == nullptr || target == nullptr)
                return;

            optimize_set[index].remove(explore_node);
            //explore_node->setTreeNodeState(TreeNodeState::not_in_set);

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
        }*/


    private:
        std::shared_ptr<ReferencePath> reference_path;
        std::vector<int> _dominant_index;

    };
}



#endif //NANOWIREPLANNER_NANOWIRE_REF_ISST_HPP
