//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_GLOBAL_PATH_HPP
#define NANOWIREPLANNER_GLOBAL_PATH_HPP
#include "acado/acado_toolkit.hpp"
#include "config/nanowire_config.hpp"
#include "config/planner_config.hpp"
#include <boost/filesystem.hpp>
#include "nanowire_utility.hpp"
#include <fstream>
#include "global_route.hpp"
namespace acsr {

    USING_NAMESPACE_ACADO
    template <int NANOWIRE_COUNT>
    class GlobalPath {
    public:
        GlobalPath() = default;
        virtual ~GlobalPath() = default;

        /***
         * initialize
         * @param wire_count nanowire count
         * @param init_state init state
         * @param target_state target state
         * @return
         */
        bool init(const Eigen::Matrix<double,2*NANOWIRE_COUNT,1> &init_state, const Eigen::Matrix<double,2*NANOWIRE_COUNT,1> &target_state) {
            ///read reference speed file
            if(NanowireConfig::type=="cc600") {
                if (!boost::filesystem::exists("data/reference_speed_cc600.txt")) {
                    std::cout << "Reference data for cc600 is missing";
                    return false;
                }
                ref_time.read("data/reference_speed_cc600.txt");
            }else if(NanowireConfig::type=="cc400") {
                if (!boost::filesystem::exists("data/reference_speed_cc400_h700.txt")) {
                    std::cout << "Reference data for cc400 is missing";
                    return false;
                }
                ref_time.read("data/reference_speed_cc400_h200.txt");
                for (auto i = 0; i < ref_time.getNumPoints(); ++i) {
                    ref_time.setVector(i,ref_time.getVector(i)*1e-6);
                }
            }else if(NanowireConfig::type=="cc60"){
                if (!boost::filesystem::exists("data/reference_speed_cc400_h700.txt")) {
                    std::cout << "Reference data for cc60 is missing";
                    return false;
                }
                ref_time.read("data/reference_speed_cc400_h700.txt");
                for (auto i = 0; i < ref_time.getNumPoints(); ++i) {
                    ref_time.setVector(i,ref_time.getVector(i)*1e-6);
                    ref_time.setTime(i,ref_time.getTime(i)*1.5);
                }
            }

            //_n_wires = wire_count;
            _init_state = init_state;
            _target_state = target_state;

            ///for high number nanowire system, decrease the reference speed
            double multiply = 1.0;
            if (_init_state.size()/2 > 3) {
                multiply = multiply + 0.30 * (_init_state.size()/2 - 2);
                for (auto i = 0; i < ref_time.getNumPoints(); ++i) {
                    ref_time.setTime(i, multiply * ref_time.getTime(i));
                }
            }
            return true;
        }

        /***
         * get start and target electrodes
         */
        void getStartTargetElectordes(){
            std::vector<ElectordePositionType> start_vectors;
            std::vector<ElectordePositionType> target_vectors;
            std::vector<Eigen::Vector2d> init_state_vectors,target_state_vectors;
            for(auto i=0;i<NANOWIRE_COUNT;++i){
                init_state_vectors.emplace_back(_init_state(2*i),_init_state(2*i+1));
                target_state_vectors.emplace_back(_target_state(2*i),_target_state(2*i+1));
            }

            ///get nearest electrodes around init state
            auto temp_start_vectors = NanowireConfig::getNearElectrodes(init_state_vectors.front(),PlannerConfig::nearest_electrode_count);
            std::for_each(temp_start_vectors.begin(),temp_start_vectors.end(),[&](const Eigen::Vector2i& pt){
                start_vectors.push_back(ElectordePositionType{pt});
            });
            ///generate init electrodes state
            for(uint i=1;i<NANOWIRE_COUNT;++i){
                auto temp = NanowireConfig::getNearElectrodes(init_state_vectors[i],1);
                start_vectors = combineVectors(start_vectors,temp);
            }

            ///get nearest electrodes around target state
            auto temp_target_vectors = NanowireConfig::getNearElectrodes(target_state_vectors.front(),PlannerConfig::nearest_electrode_count);
            std::for_each(temp_target_vectors.begin(),temp_target_vectors.end(),[&](const Eigen::Vector2i& pt){
                target_vectors.push_back(ElectordePositionType{pt});
            });
            ///generate target electrodes state
            for(uint i=1;i<NANOWIRE_COUNT;++i){
                auto temp = NanowireConfig::getNearElectrodes(target_state_vectors[i],1);
                target_vectors = combineVectors(target_vectors,temp);
            }
            ///generate start-target pairs
            std::vector<std::pair<ElectordePositionType,ElectordePositionType>> p;
            for(const auto& s:start_vectors) {
                for (const auto &t:target_vectors) {
                    p.emplace_back(s, t);
                }
            }

            ///sort by path length
            std::sort(p.begin(),p.end(),[&](const std::pair<ElectordePositionType,ElectordePositionType>& elem1,const std::pair<ElectordePositionType,ElectordePositionType>& elem2){
                return getHeuristic(NANOWIRE_COUNT,elem1.first,elem2.second)<getHeuristic(NANOWIRE_COUNT,elem2.first,elem2.second);
            });

            ///slim to maximum 5
            global_start_target.clear();
            if(p.size()<5){
                std::copy(p.begin(),p.end(),std::back_inserter(global_start_target));
            }else{
                std::copy(p.begin(),p.begin()+5,std::back_inserter(global_start_target));
            }

            ///write to file
            /// format:
            /*
            start: e1x,e1y,e2x,e2y,...,enx,eny
            target:
            start:
            target:
            ...
             */
            std::ofstream out("electrodes_for_planner.txt");
            int i=0;
            auto it = p.begin();
            for(;i<15&&it!=p.end();++it,++i){
                for(auto& elem:it->first){
                    out<<elem(0)<<" "<<elem(1)<<" ";
                }
                out<<'\n';
                for(auto& elem:it->second){
                    out<<elem(0)<<" "<<elem(1)<<" ";
                }
                out<<'\n';
            }
            out.close();
        }

        /***
         * generate global path
         * @return global path
         */
        std::vector<std::vector<ElectordePositionType>> generateGlobalPath(){
            std::vector<std::vector<ElectordePositionType>> electrodes_paths;
            const std::vector<std::vector<int>> divided_vecs{{1},{2},{3},{4},{3,2},{3,3},{4,3},{4,4},{3,3,3},{3,3,4},{3,4,4},{4,4,4},{4,3,3,3},{4,4,3,3}};
            for(auto & k : global_start_target) {
                global::GlobalRoute global_route;
                global_route.init();
                global::NanowirePositionType init_state;
                global::NanowirePositionType target_state;
                for (auto i = 0; i < NANOWIRE_COUNT; ++i) {
                    init_state.push_back(std::make_pair(k.first[i](0),k.first[i](1)));
                    target_state.push_back(std::make_pair(k.second[i](0),k.second[i](1)));
                }
                auto init_index = global::electrodeVectorToIndex(NANOWIRE_COUNT, init_state);
                auto target_index = global::electrodeVectorToIndex(NANOWIRE_COUNT, target_state);
                std::vector<global::IndexType> path;
                if (global_route.constructTree(NANOWIRE_COUNT, init_index, target_index, divided_vecs[NANOWIRE_COUNT - 1])) {
                    path = global_route.getBestSolution();
                    std::vector<ElectordePositionType> electrode_path;
                    std::transform(path.rbegin(),path.rend(),std::back_inserter(electrode_path),[&](global::IndexType index){
                        auto v = global::indexToElectrodeVector(NANOWIRE_COUNT,index);
                        ElectordePositionType ep;
                        for(auto&e:v){
                            ep.push_back(Eigen::Vector2i(e.first,e.second));
                        }
                        return ep;
                    });

                    for (auto& i :electrode_path){
                        for(auto& j:i){
                            std::cout<<j(0)<<' '<<j(1)<<' ';
                        }
                        std::cout<<'\n';
                    }
                    electrodes_paths.push_back(electrode_path);
                } else {
                    std::cout << "cannot find solution\n";
                    continue;
                }
                std::cout<<"----------------------------------------------\n";
            }
            return electrodes_paths;
        }

        /***
         * generate best reference trajectory
         * @param states trajectory
         */
        void generateBestReferenceTrajectory(VariablesGrid& states){
            auto global_paths = generateGlobalPath();
            double max_time = std::numeric_limits<double>::max();
            ///select minimum cost path
            for(auto& global_path:global_paths ){
                VariablesGrid _states;
                generateReferenceTrajectory(global_path,_states);
                if(_states.getLastTime()<max_time){
                    states = _states;
                    max_time = _states.getLastTime();
                }
            }
            std::ofstream out("reference_path.txt");
            out<<std::setw(8)<<"#time";
            auto cols=states.getFirstVector().size();
            for(auto i=0;i<cols/2;i++){
                out<<"\tx"<<i<<"\ty"<<i;
            }
            out<<"\n";

            for (auto i=0;i<states.getNumPoints();++i) {
                out<<std::setw(8)<<states.getTime(i);
                DVector v=states.getVector(i);
                for (auto j=0;j<cols;j++) {
                    out<<"\t"<<std::setw(16)<<v(j);
                }
                out<<"\n";
            }
            out.close();
        }

        /***
         * generate reference trajectory
         * @param electrodes_path
         * @param states
         */
        void generateReferenceTrajectory(const std::vector<ElectordePositionType>& electrodes_path,VariablesGrid& states)
        {
            //init segments
            states.init();
            generateInitSegmentTrajectory(electrodes_path.front(),states);

            //middle segments
            for (uint i=0;i<electrodes_path.size()-1;++i) {
                double init_time = states.getLastTime()+2.0;
                VariablesGrid temp_state;
                generateIntermediaSegmentTrajectory(electrodes_path[i],electrodes_path[i+1],temp_state);
                for(uint j=0;j<temp_state.getNumPoints();++j){
                    states.addVector(temp_state.getVector(j),init_time+temp_state.getTime(j));
                }
            }

            //last segments
            double init_time = states.getLastTime()+2.0;
            VariablesGrid temp_state;
            generateFinalSegmentTrajectory(electrodes_path.back(),temp_state);
            for(uint j=0;j<temp_state.getNumPoints();++j){
                states.addVector(temp_state.getVector(j),init_time+temp_state.getTime(j));
            }
        }


    private:
        VariablesGrid ref_time;
        //int _n_wires;
        Eigen::Matrix<double,2*NANOWIRE_COUNT,1> _init_state, _target_state;
        std::vector<std::pair<ElectordePositionType,ElectordePositionType>> global_start_target;

    private:
        /***
         * increase state dimension (add one electrode to existing states)
         * @param vectors existing states
         * @param pts new state
         * @return combined state
         */
        std::vector<ElectordePositionType> combineVectors(const std::vector<ElectordePositionType> &vectors, ElectordePositionType pts){
            std::vector<ElectordePositionType> new_vectors;
            for(auto v : vectors){
                for(auto & pt : pts){
                    auto k = v;
                    k.push_back(pt);
                    new_vectors.push_back(k);
                }
            }
            return new_vectors;
        }

        /***
         * generate init segment trajectory (start state to start electrode) for one nanowire
         * @param nanowire_position start state
         * @param electrode_position start electrode
         * @param single_state output states
         * @return total cost for output states
         */
        double generateSingleInitSegmentTrajectory(const Eigen::Vector2d& nanowire_position,const Eigen::Vector2i& electrode_position,VariablesGrid &single_state)
        {
            const auto d_start = (NanowireConfig::electrodePositionToPosition(electrode_position)-nanowire_position).norm();
            uint index_start;
            for (index_start = 0; index_start<ref_time.getNumPoints();++index_start) {
                if(NanowireConfig::electrodes_space-ref_time.getVector(index_start)(0)<d_start)
                    break;
            }
            auto init_time = ref_time.getTime(index_start);
            single_state.init();
            single_state.addVector(nanowire_position,0.0);

            for (auto j = index_start+1; j<ref_time.getNumPoints();++j) {
                DVector vec = nanowire_position + (d_start-(NanowireConfig::electrodes_space-ref_time.getVector(j)(0))) / d_start * (NanowireConfig::electrodePositionToPosition(electrode_position)-nanowire_position);
                single_state.addVector(vec,ref_time.getTime(j)-init_time);
            }
            return d_start;
        }

        /***
         * generate final segment trajectory (last electrode to target state) for one nanowire
         * @param electrode_position last electrode
         * @param nanowire_position target state
         * @param single_state output states
         * @return total cost for output states
         */
        double generateSingleFinalSegmentTrajectory(const Eigen::Vector2i& electrode_position,
                                                    const Eigen::Vector2d& nanowire_position,
                                                    VariablesGrid &single_state)
        {
            const auto start = NanowireConfig::electrodePositionToPosition(electrode_position);
            auto d_end = (start-nanowire_position).norm();
            single_state.init();
            uint j = 0;
            for (j = 0; j<ref_time.getNumPoints();++j) {
                if(ref_time.getVector(j)(0)>d_end)
                    break;
                DVector vec = start + ref_time.getVector(j)(0) / NanowireConfig::electrodes_space * (nanowire_position-start);
                single_state.addVector(vec,ref_time.getTime(j));
            }
            single_state.addVector(nanowire_position,ref_time.getTime(j+1));
            return d_end;
        }

        /***
         * generate intermedia segment trajectory (electrode to electrode) for one nanowire
         * @param electrode_start 1st electrode
         * @param electrode_target 2nd electrode
         * @param single_state single_state output states
         */
        void generateSingleIntermediaSegmentTrajectory(const Eigen::Vector2i& electrode_start,
                                                       const Eigen::Vector2i& electrode_target,
                                                       VariablesGrid &single_state)
        {
            const auto start = NanowireConfig::electrodePositionToPosition(electrode_start);
            const auto target = NanowireConfig::electrodePositionToPosition(electrode_target);
            single_state.init();
            for (auto j = 0; j<ref_time.getNumPoints();++j) {
                DVector vec = start + ref_time.getVector(j)(0) / NanowireConfig::electrodes_space * (target-start);
                single_state.addVector(vec,ref_time.getTime(j));
            }
        }

        /***
         * generate init segment trajectory for all nanowire
         * @param electrode start electrodes vector
         * @param state output states
         * @return total cost for output states
         */
        double generateInitSegmentTrajectory(const ElectordePositionType &electrode,VariablesGrid &state){
            state.init();
            std::vector<VariablesGrid> grids;

            double d_start = 0.0;
            uint index = 0;

            for(auto i=0;i<NANOWIRE_COUNT;++i){
                VariablesGrid single_state;
                auto d = generateSingleInitSegmentTrajectory(Eigen::Vector2d(_init_state(2*i),_init_state(2*i+1)),electrode[i],single_state);
                if(d>d_start){
                    d_start=d;
                    index=i;
                }
                grids.push_back(single_state);
            }

            for (auto i=0;i<grids[index].getNumPoints();++i) {
                DVector s(2*grids.size());
                for(uint j=0;j<grids.size();++j){
                    auto v =grids[j].linearInterpolation(grids[index].getTime(i));
                    s(2*j)=v(0);
                    s(2*j+1)=v(1);
                }
                state.addVector(s,grids[index].getTime(i));
            }
            return d_start;
        }

        /***
         * generate final segment trajectory for all nanowire
         * @param electrode target electrodes vector
         * @param state output states
         * @return total cost for output states
         */
        double generateFinalSegmentTrajectory(const ElectordePositionType &electrode, VariablesGrid &state)
        {
            state.init();
            std::vector<VariablesGrid> grids;

            double d_start = 0.0;
            uint index = 0;

            for(auto i=0;i<electrode.size();++i){
                VariablesGrid single_state;
                auto d = generateSingleFinalSegmentTrajectory(electrode[i],Eigen::Vector2d(_target_state(2*i),_target_state(2*i+1)),single_state);
                if(d>d_start){
                    d_start=d;
                    index=i;
                }
                grids.push_back(single_state);
            }

            for (uint i=0;i<grids[index].getNumPoints();++i) {
                DVector s(2*grids.size());
                for(uint j=0;j<grids.size();++j){
                    auto v =grids[j].linearInterpolation(grids[index].getTime(i));
                    s(2*j)=v(0);
                    s(2*j+1)=v(1);
                }
                state.addVector(s,grids[index].getTime(i));
            }
            return d_start;
        }

        /***
         * generate intermedia segment trajectory (cell edge) for all nanowire
         * @param electrode_start start electrodes vector
         * @param electrode_target target electrodes vector
         * @param state output states
         */
        void generateIntermediaSegmentTrajectory(const ElectordePositionType &electrode_start,
                                                 const ElectordePositionType &electrode_target,
                                                 VariablesGrid &state)
        {
            state.init();
            std::vector<VariablesGrid> grids;

            for(uint i=0;i<electrode_start.size();++i){
                VariablesGrid single_state;
                generateSingleIntermediaSegmentTrajectory(electrode_start[i],electrode_target[i],single_state);
                grids.push_back(single_state);
            }

            for (uint i=0;i<grids[0].getNumPoints();++i) {
                DVector s(2*grids.size());
                for(uint j=0;j<grids.size();++j){
                    auto v =grids[j].linearInterpolation(grids[0].getTime(i));
                    s(2*j)=v(0);
                    s(2*j+1)=v(1);
                }
                state.addVector(s,grids[0].getTime(i));
            }
        }


    };
}


#endif //NANOWIREPLANNER_GLOBAL_PATH_HPP
