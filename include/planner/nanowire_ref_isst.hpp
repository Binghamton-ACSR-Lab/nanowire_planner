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

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    struct PropagateParameters{
        Eigen::Matrix<double,STATE_DIMENSION,1> state;
        Eigen::Matrix<double,CONTROL_DIMENSION,1> control;
        double duration;
    } ;

    template <int STATE_DIMENSION,int CONTROL_DIMENSION>
    class RefSST: public SST<STATE_DIMENSION,CONTROL_DIMENSION>, virtual public Planner<STATE_DIMENSION,CONTROL_DIMENSION>{

        using StateType = Eigen::Matrix<double,STATE_DIMENSION,1>;
        using ControlType = Eigen::Matrix<double,CONTROL_DIMENSION,1>;

        using TreeNodeType = TreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>;
        using SSTTreeNodeType = SSTTreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>;

        using NodePtr = std::shared_ptr<Node<double,STATE_DIMENSION>>;
        using TreeNodePtr = std::shared_ptr <TreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>>;
        using SSTTreeNodePtr = std::shared_ptr <SSTTreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>>;

    protected:
        /***
         * select a node to explore, unlike in SST or RRT, this process is not stochastic. we select a node with least estimated solution cost
         * @param tree_id
         * @param parent
         * @return
         */
        bool searchSelection(TreeId tree_id, TreeNodePtr& parent){
            auto p = randomDouble(0.0,1.0);
            parent = nullptr;
            if(p<PlannerConfig::search_p){
                auto point = getRandomReferencePoint();
                auto near = this->getNearNodeByRadius(point,tree_id,PlannerConfig::sst_delta_near);
                if(!near.empty()) {
                    auto it = std::min_element(near.begin(), near.end(),
                                               [](const NodePtr &node1,const NodePtr &node2) {
                                                   return std::static_pointer_cast<TreeNodeType>(node1)->getCost() < std::static_pointer_cast<TreeNodeType>(node2)->getCost();
                                               });
                    parent = std::static_pointer_cast<TreeNodeType>(*it);
                }else{
                    parent = std::static_pointer_cast<TreeNodeType>(this->getNearNodeByCount(point,tree_id,1).front());
                }

            }else{
                auto point = this->_dynamic_system->randomState();
                auto near = this->getNearNodeByCount(point,tree_id,10);
                auto it = std::min_element(near.begin(), near.end(),
                                           [](const NodePtr &node1, const NodePtr &node2) {
                                               return std::static_pointer_cast<TreeNodeType>(node1)->getCost() < std::static_pointer_cast<TreeNodeType>(node2)->getCost();
                                           });
                parent = std::static_pointer_cast<TreeNodeType>(*it);
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
        bool forwardBlossom(const TreeNodePtr & parent,std::vector<PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& params){

            if(parent->getTreeId()!=TreeId::forward || !std::static_pointer_cast<SSTTreeNodeType>(parent)->isActive())
                return false;
            const int n=3;
            std::map<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>> quality_map;
            std::mutex map_mutex;
            std::vector<std::thread> thread_pool;
            for(auto i=0;i<n-1;++i){
                thread_pool.push_back(std::thread(&acsr::RefSST<STATE_DIMENSION,CONTROL_DIMENSION>::forwardBlossomThread,this,parent,PlannerConfig::blossomM/n,std::ref(quality_map),std::ref(map_mutex)));
            }
            forwardBlossomThread(parent,PlannerConfig::blossomM/n,quality_map,map_mutex);
            for(auto&t:thread_pool){
                if(t.joinable())t.join();
            }
            if(quality_map.empty())return false;
            int i = 0;
            for(auto it = quality_map.rbegin();i<PlannerConfig::blossomN && it!=quality_map.rend();++it,++i){
                params.push_back(it->second);
            }
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
        bool backwardBlossom(const TreeNodePtr& parent,std::vector<PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& params){

            if(parent->getTreeId()!=TreeId::backward || !std::static_pointer_cast<SSTTreeNodeType>(parent)->isActive())
                return false;
            const int n=3;
            std::map<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>> quality_map;
            std::mutex map_mutex;
            std::vector<std::thread> thread_pool;
            for(auto i=0;i<n-1;++i){
                thread_pool.push_back(std::thread(&acsr::RefSST<STATE_DIMENSION,CONTROL_DIMENSION>::backwardBlossomThread,this,parent,PlannerConfig::blossomM/n,std::ref(quality_map),std::ref(map_mutex)));
            }
            backwardBlossomThread(parent,PlannerConfig::blossomM/n,quality_map,map_mutex);
            for(auto&t:thread_pool){
                if(t.joinable())t.join();
            }
            if(quality_map.empty())return false;
            int i = 0;
            for(auto it = quality_map.rbegin();i<PlannerConfig::blossomN && it!=quality_map.rend();++it,++i){
                params.push_back(it->second);
            }
            return true;
        }

        void backwardBlossomThread(const TreeNodePtr& parent,int iterate_n,std::map<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& quality_map,std::mutex& map_mutex){
            StateType temp_state;
            //std::map<double,PropagateParameters> quality_map;
            double temp_duration;

            for(int i=0;i<iterate_n;i++) {
                if(!this->_run_flag)return;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->backwardPropagateBySteps(parent->getState(),temp_control,
                                                                    steps,temp_state,temp_duration)){
                    std::scoped_lock<std::mutex> lock(map_mutex);
                    quality_map[getQuality(parent->getCost() + temp_duration,temp_state,TreeId::backward)] =
                            PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> {temp_state,temp_control,temp_duration};
                }
            }
        }

        void forwardBlossomThread(const TreeNodePtr& parent,int iterate_n,std::map<double,PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>>& quality_map,std::mutex& map_mutex){
            StateType temp_state;
            double temp_duration;
            for(int i=0;i<iterate_n;i++) {
                if(!this->_run_flag)return;
                auto temp_control = this->_dynamic_system->randomControl();
                auto steps = randomInteger(PlannerConfig::min_time_steps,PlannerConfig::max_time_steps);
                if (this->_dynamic_system->forwardPropagateBySteps(parent->getState(),temp_control,
                                                                   steps,temp_state,temp_duration)){
                    std::scoped_lock<std::mutex> lock(map_mutex);
                    quality_map[getQuality(parent->getCost() + temp_duration,temp_state,TreeId::forward)] =
                            PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION> {temp_state,temp_control,temp_duration};
                }
            }
        }

        StateType getRandomReferencePoint(){
            std::uniform_int_distribution<int> distribution(0,reference_path->getStates().getNumPoints()-1);
            auto index = distribution(_random_engine);
            return reference_path->getStates().getVector(index);
        }

        double getQuality(double time,  const StateType &state, const TreeId treeid){
            double value;
            auto max_time = reference_path->getMaxTime();
            StateType reference_state;
            if(treeid==TreeId::forward)
                reference_state= reference_path->getState(time);
            else if(treeid==TreeId::backward){
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
            for(auto i=0;i<STATE_DIMENSION/2;++i){
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
            auto z = std::exp(-time);
            value = z/(dominant_ref-dominant_state).norm();

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
        explicit RefSST(std::shared_ptr<NanowireSystem<STATE_DIMENSION/2,CONTROL_DIMENSION>> dynamic_system):SST<STATE_DIMENSION,CONTROL_DIMENSION>(dynamic_system),
                                                                        Planner<STATE_DIMENSION,CONTROL_DIMENSION>(dynamic_system){
        }

        RefSST()=delete;
        RefSST(const RefSST&) = delete;

        ~RefSST() override{
            this->forward_rtree.clear();
            this->backward_rtree.clear();
            this->optimize_set.clear();
        }

        /***
         * setup
         */
        void setup() override {
            SST<STATE_DIMENSION,CONTROL_DIMENSION>::setup();
            ///generating global reference path & storing it to "reference_path.txt"
            GlobalPath global_path;
            global_path.init(STATE_DIMENSION/2,this->_init_state,this->_target_state);
            global_path.getStartTargetElectordes();
            VariablesGrid states;
            global_path.generateBestReferenceTrajectory(states);

            ///read reference path
            reference_path = std::make_shared<ReferencePath<STATE_DIMENSION/2>>();
            reference_path->readFile("reference_path.txt");

            ///check dominant path
            std::vector<double> length(STATE_DIMENSION/2,0.0);
            for(auto j=0;j<states.getNumPoints()-1;++j){
                for(auto i=0;i<STATE_DIMENSION/2;++i){
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
            auto v = std::min(PlannerConfig::dominant_path_count,STATE_DIMENSION/2);
            for(auto i=0;i<v;++i)
                _dominant_index.push_back(length_with_index[i].first);
        }

        /***
         * override forward step
         */
        void forwardStep() override{
            ///select a node to be explored in forward tree
            TreeNodePtr parent= nullptr;
            if(!searchSelection(TreeId::forward,parent))return;
            if(!std::static_pointer_cast<SSTTreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>>(parent)->isActive())
                return;
            std::vector<PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>> params;
            if (forwardBlossom(parent,params)){
                if(parent==nullptr)
                    return;
                for(auto param:params){
                    auto new_node = this->addToTree(TreeId::forward,parent,param.state,param.control,param.duration);
                    if(PlannerConfig::show_node && new_node!= nullptr){
                        auto state = new_node->getState();
                        std::thread t([this,state](){
                            Eigen::VectorXd s(2*_dominant_index.size());
                            for(auto i=0;i<_dominant_index.size();++i){
                                s[2*i] = state[2*_dominant_index[i]];
                                s[2*i+1] = state[2*_dominant_index[i]+1];
                            }
                            this->notifyNodeAdded(state,TreeId::forward);
                        });
                        t.detach();
                    }
                    this->checkConnection(new_node);
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
            if(!std::static_pointer_cast<SSTTreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>>(parent)->isActive())return;

            std::vector<PropagateParameters<STATE_DIMENSION,CONTROL_DIMENSION>> params;
            if (backwardBlossom(parent,params)){
                ///add parent to close map
                if(parent==nullptr || !std::static_pointer_cast<SSTTreeNode<double,STATE_DIMENSION,CONTROL_DIMENSION>>(parent)->isActive())
                    return;
                for(auto& param:params){
                    auto new_node = this->addToTree(TreeId::backward,parent,param.state,param.control,param.duration);
                    if(PlannerConfig::show_node && new_node!= nullptr){
                        auto state = new_node->getState();
                        std::thread t([this,state](){
                            Eigen::VectorXd s(2*_dominant_index.size());
                            for(auto i=0;i<_dominant_index.size();++i){
                                s[2*i] = state[2*_dominant_index[i]];
                                s[2*i+1] = state[2*_dominant_index[i]+1];
                            }
                            this->notifyNodeAdded(state,TreeId::backward);
                        });
                        t.detach();

                    }
                    this->checkConnection(new_node);
                }
            }
        }

    protected:
        std::shared_ptr<ReferencePath<STATE_DIMENSION/2>> reference_path;
        std::vector<int> _dominant_index;
    };
}

#endif //NANOWIREPLANNER_NANOWIRE_REF_ISST_HPP
