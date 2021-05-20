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
        /***
         * setup
         */
        void setup(const std::shared_ptr<NanowireConfig>& nanowire_config) override {
            SST::setup(nanowire_config);
            ///generating global reference path & storing it to "reference_path.txt"
            GlobalPath global_path;
            global_path.init(_init_state,_target_state,nanowire_config);
            global_path.getStartTargetElectordes();
            VariablesGrid states;
            global_path.generateBestReferenceTrajectory(states);

            ///read reference path
            reference_path = std::make_shared<ReferencePath>();
            reference_path->readFile("reference_path.txt");

            ///check dominant path
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
                length_with_index.emplace_back(i,length[i]);
            std::sort(length_with_index.begin(),length_with_index.end(),[](const auto& p1,const auto& p2){
                return p1.second>p2.second;
            });
            auto v = std::min(PlannerConfig::dominant_path_count,_dynamic_system->getRobotCount());
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
            
            std::uniform_real_distribution<double> distribution(0.0,1.0);
            auto p = distribution(_random_engine);
            parent = nullptr;
            if(p<PlannerConfig::search_p){
                auto point = getRandomReferencePoint();
                auto near = getNearNodeByRadiusAndNearest(point,tree_id,PlannerConfig::sst_delta_near);
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

            ///propagate M times and select a best node with least estimated solution cost
            for(int i=0;i<PlannerConfig::blossomM;i++) {
                if(!_run_flag)return false;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,PlannerConfig::integration_step,
                                                                   steps,temp_state,temp_duration)){
                    return_value = true;
                    quality_map[getQuality(parent->getCost() + temp_duration,temp_state,TreeId::forward)] =
                            PropagateParameters {temp_state,temp_control,temp_duration};
                }
            }
            if(return_value){
                int i = 0;
                for(auto it = quality_map.rbegin();i<PlannerConfig::blossomN && it!=quality_map.rend();++it,++i){
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

            for(int i=0;i<PlannerConfig::blossomM;i++) {
                if(!_run_flag)return false;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->reversePropagateBySteps(parent->getState(),temp_control,PlannerConfig::integration_step,
                                                                   steps,temp_state,temp_duration)){
                    return_value = true;
                    quality_map[getQuality(parent->getCost() + temp_duration,temp_state,TreeId::reverse)] =
                            PropagateParameters {temp_state,temp_control,temp_duration};
                }
            }
            if(return_value){
                int i = 0;
                for(auto it = quality_map.rbegin();i<PlannerConfig::blossomN && it!=quality_map.rend();++it,++i){
                    params.push_back(it->second);
                }
            }
            return return_value;
        }

        Eigen::VectorXd getRandomReferencePoint(){
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
                    k++;
                }
            }

            if(time <= max_time){
                auto q = (time/max_time-1.0)*PlannerConfig::quality_factor;
                auto z = std::exp(q);
                value = z/(dominant_ref-dominant_state).norm();//+0.001 * z/(non_dominant_ref-non_dominant_state).norm();
            }else {
                auto q = PlannerConfig::quality_decrease_factor*(max_time/time-1.0)*PlannerConfig::quality_factor;
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
            this->forward_tree.clear();
            this->reverse_tree.clear();
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
                    if(PlannerConfig::show_node && new_node!= nullptr){
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
                    if(PlannerConfig::show_node && new_node!= nullptr){
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

    protected:
        std::shared_ptr<ReferencePath> reference_path;
        std::vector<int> _dominant_index;
    };
}

#endif //NANOWIREPLANNER_NANOWIRE_REF_ISST_HPP
