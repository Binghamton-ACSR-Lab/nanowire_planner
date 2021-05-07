//
// Created by acsr on 4/27/21.
//

#ifndef NANOWIREPLANNER_GLOBAL_PATH_HPP
#define NANOWIREPLANNER_GLOBAL_PATH_HPP
#include "acado/acado_toolkit.hpp"
#include "nanowire_config.hpp"
#include <boost/filesystem.hpp>
#include "nanowire_utility.hpp"
#include <fstream>
#include "global_route.hpp"
namespace acsr {

    USING_NAMESPACE_ACADO
    class GlobalPath {
    public:
        GlobalPath() = default;

        virtual ~GlobalPath() = default;

        bool init(const Eigen::VectorXd &init_state, const Eigen::VectorXd &target_state,std::shared_ptr<NanowireConfig> nanowire_config) {
            if (!boost::filesystem::exists("data/reference cost.txt"))
                return false;
            ref_time.read("data/reference cost.txt");

            _n_wires = nanowire_config->getNanowireCount();
            _init_state = init_state;
            _target_state = target_state;
            _nanowire_config = nanowire_config;

            double multiply = 1.0;
            if (_init_state.size()/2 > 3) {
                multiply = multiply + 0.30 * (_init_state.size()/2 - 2);

                for (auto i = 0; i < ref_time.getNumPoints(); ++i) {
                    ref_time.setTime(i, multiply * ref_time.getTime(i));
                }
            }



            //nanowire_config = std::make_shared<NanowireConfig>();
            //nanowire_config->readFile()

            //global_paths.clear();
            //states_vectors.clear();


            //electrodes_ends = std::map<double, std::pair<std::vector<electrode_position_t>, std::vector<electrode_position_t>>>();

            //generateGlobalPaths();
        }

        void getStartTargetElectordes(){
            std::vector<ElectordePositionType> start_vectors;
            std::vector<ElectordePositionType> target_vectors;
            std::vector<Eigen::Vector2d> init_state_vectors,target_state_vectors;
            for(auto i=0;i<_n_wires;++i){
                init_state_vectors.push_back({_init_state(2*i),_init_state(2*i+1)});
                target_state_vectors.push_back({_target_state(2*i),_target_state(2*i+1)});
            }

            auto temp_start_vectors = _nanowire_config->getNearElectrodes(init_state_vectors.front());

            std::for_each(temp_start_vectors.begin(),temp_start_vectors.end(),[&](const Eigen::Vector2i& pt){
                start_vectors.push_back(ElectordePositionType{pt});
            });

            for(uint i=1;i<_n_wires;++i){
                auto temp = _nanowire_config->getNearElectrodes(init_state_vectors[i]);
                start_vectors = combineVectors(start_vectors,temp);
            }

            auto temp_target_vectors = _nanowire_config->getNearElectrodes(target_state_vectors.front());
            std::for_each(temp_target_vectors.begin(),temp_target_vectors.end(),[&](const Eigen::Vector2i& pt){
                target_vectors.push_back(ElectordePositionType{pt});
            });
            for(uint i=1;i<_n_wires;++i){
                auto temp = _nanowire_config->getNearElectrodes(target_state_vectors[i]);
                target_vectors = combineVectors(target_vectors,temp);
            }

            std::vector<std::pair<ElectordePositionType,ElectordePositionType>> p;
            for(const auto& s:start_vectors) {
                for (const auto &t:target_vectors) {
                    p.emplace_back(s, t);
                }
            }

            std::sort(p.begin(),p.end(),[&](const std::pair<ElectordePositionType,ElectordePositionType>& elem1,const std::pair<ElectordePositionType,ElectordePositionType>& elem2){
                return getHeuristic(_n_wires,elem1.first,elem2.second)<getHeuristic(_n_wires,elem2.first,elem2.second);
            });

            global_start_target.clear();
            if(p.size()<15){
                std::copy(p.begin(),p.end(),std::back_inserter(global_start_target));
            }else{
                std::copy(p.begin(),p.begin()+15,std::back_inserter(global_start_target));
            }
            /*
            auto v = *std::min_element(p.begin(),p.end(),[&](const std::pair<ElectordePositionType,ElectordePositionType>& elem1,const std::pair<ElectordePositionType,ElectordePositionType>& elem2){
                return getHeuristic(_n_wires,elem1.first,elem2.second)<getHeuristic(_n_wires,elem2.first,elem2.second);
            });*/
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
            //return v;
        }

        std::vector<std::vector<ElectordePositionType>> generateGlobalPath(){
            std::vector<std::vector<ElectordePositionType>> electrodes_paths;
            const std::vector<std::vector<int>> divided_vecs{{2},{3},{4},{3,2},{3,3},{4,3},{4,4},{3,3,3},{3,3,4},{3,4,4},{4,4,4},{4,3,3,3},{4,4,3,3}};
            for(auto k=0;k<global_start_target.size();++k) {

                global::GlobalRoute global_route;
                global_route.init();
                //std::cout << "Iteration: " << k << std::endl;
                global::NanowirePositionType init_state;
                global::NanowirePositionType target_state;
                for (auto i = 0; i < _nanowire_config->getNanowireCount(); ++i) {
                    init_state.push_back(std::make_pair(global_start_target[k].first[i](0),global_start_target[k].first[i](1)));
                    target_state.push_back(std::make_pair(global_start_target[k].second[i](0),global_start_target[k].second[i](1)));
                }
                auto init_index = global::electrodeVectorToIndex(_n_wires, init_state);
                auto target_index = global::electrodeVectorToIndex(_n_wires, target_state);
                std::vector<global::IndexType> path;


                if (global_route.constructTree(_n_wires, init_index, target_index, divided_vecs[_n_wires - 2])) {
                    path = global_route.getBestSolution();
                    std::vector<ElectordePositionType> electrode_path;
                    for(auto it=path.rbegin();it!=path.rend();++it){

                    }
                    std::transform(path.rbegin(),path.rend(),std::back_inserter(electrode_path),[&](global::IndexType index){
                        auto v = global::indexToElectrodeVector(_n_wires,index);
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

        void generateBestReferenceTrajectory(VariablesGrid& states){
            auto global_paths = generateGlobalPath();
            double max_time = std::numeric_limits<double>::max();
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
            int cols=states.getFirstVector().size();
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
        int _n_wires;
        Eigen::VectorXd _init_state, _target_state;
        std::shared_ptr<NanowireConfig> _nanowire_config;
        std::vector<std::pair<ElectordePositionType,ElectordePositionType>> global_start_target;

    private:
        std::vector<ElectordePositionType> combineVectors(const std::vector<ElectordePositionType> &vectors, ElectordePositionType pts)
        {
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


        double generateSingleInitSegmentTrajectory(const Eigen::Vector2d& nanowire_position,const Eigen::Vector2i& electrode_position,VariablesGrid &single_state)
        {
            const auto d_start = (_nanowire_config->electrodePositionToPosition(electrode_position)-nanowire_position).norm();
            uint index_start;
            for (index_start = 0; index_start<ref_time.getNumPoints();++index_start) {
                if(600e-6-ref_time.getVector(index_start)(0)<d_start)
                    break;
            }
            auto init_time = ref_time.getTime(index_start);
            single_state.init();
            single_state.addVector(nanowire_position,0.0);

            for (auto j = index_start+1; j<ref_time.getNumPoints();++j) {
                DVector vec = nanowire_position + (d_start-(600e-6-ref_time.getVector(j)(0))) / d_start * (_nanowire_config->electrodePositionToPosition(electrode_position)-nanowire_position);
                single_state.addVector(vec,ref_time.getTime(j)-init_time);
            }
            return d_start;
        }

        double generateSingleFinalSegmentTrajectory(const Eigen::Vector2i& electrode_position,
                                                    const Eigen::Vector2d& nanowire_position,
                                                    VariablesGrid &single_state)
        {
            const auto start = _nanowire_config->electrodePositionToPosition(electrode_position);
            auto d_end = (start-nanowire_position).norm();
            single_state.init();
            uint j = 0;
            for (j = 0; j<ref_time.getNumPoints();++j) {
                if(ref_time.getVector(j)(0)>d_end)
                    break;
                DVector vec = start + ref_time.getVector(j)(0) / 600e-6 * (nanowire_position-start);
                single_state.addVector(vec,ref_time.getTime(j));
            }
            single_state.addVector(nanowire_position,ref_time.getTime(j+1));
            return d_end;
        }

        void generateSingleIntermediaSegmentTrajectory(const Eigen::Vector2i& electrode_start,
                                                       const Eigen::Vector2i& electrode_target,
                                                       VariablesGrid &single_state)
        {
            const auto start = _nanowire_config->electrodePositionToPosition(electrode_start);
            const auto target = _nanowire_config->electrodePositionToPosition(electrode_target);
            single_state.init();
            for (auto j = 0; j<ref_time.getNumPoints();++j) {
                DVector vec = start + ref_time.getVector(j)(0) / 600e-6 * (target-start);
                single_state.addVector(vec,ref_time.getTime(j));
            }
        }


        double generateInitSegmentTrajectory(const ElectordePositionType &electrode,
                                             VariablesGrid &state)
        {
            state.init();
            std::vector<VariablesGrid> grids;

            double d_start = 0.0;
            uint index = 0;

            for(auto i=0;i<_n_wires;++i){
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

        double generateFinalSegmentTrajectory(const ElectordePositionType &electrode, VariablesGrid &state)
        {
            state.init();
            std::vector<VariablesGrid> grids;

            double d_start = 0.0;
            uint index = 0;

            for(auto i=0;i<electrode.size();++i){
                VariablesGrid single_state;
                auto d = generateSingleFinalSegmentTrajectory(electrode[i],Eigen::Vector2d(_target_state(2*i),_target_state(2*i+1)),single_state);
                //single_state.print();
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
