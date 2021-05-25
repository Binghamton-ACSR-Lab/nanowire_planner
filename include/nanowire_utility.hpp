//
// Created by acsr on 4/29/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_UTILITY_HPP
#define NANOWIREPLANNER_NANOWIRE_UTILITY_HPP

#include <random>

namespace acsr{

    ///planner type
    enum PlannerType{
        e_SST = 0,
        e_iSST,
        e_Ref_iSST
    };

    using ElectordePositionType = std::vector<Eigen::Vector2i>;

    std::default_random_engine _random_engine;

    /***
     * generate a random integer between [_min,_max]
     * @param _min
     * @param _max
     * @return
     */
    int randomInteger(int _min, int _max) {
        std::uniform_int_distribution<int> distribution(_min, _max);
        return distribution(_random_engine);
    }

    double randomDouble(double _min, double _max) {
        std::uniform_real_distribution<double> distribution(_min,_max);
        return distribution(_random_engine);
    }

    /***
     * set a random seed for random engin
     */
    void setRandomSeed(unsigned seed) {
        _random_engine.seed(seed);
    }

    /***
     * get heuristic for global route
     * @param n_wires wire count
     * @param state1 electrode positions
     * @param state2 electrode positions
     * @return
     */
    int getHeuristic(int n_wires,const ElectordePositionType& state1,const ElectordePositionType& state2){
        int value = 0;
        for(auto i=0;i<n_wires;++i) {
            auto temp_v=(state1[i]-state2[i]).lpNorm<1>();
            value = std::max(value, temp_v);
        }
        return value;
    }

    /***
     * convert planner type to string
     * @param planner_type planner type
     * @return planner string
     */
    static std::string getPlannerString(PlannerType planner_type){
        switch (planner_type) {
            case PlannerType::e_SST:
                return "SST";
            case PlannerType::e_iSST:
                return "iSST";
            case PlannerType::e_Ref_iSST:
                return "Ref_iSST";
        }
    }

}



#endif //NANOWIREPLANNER_NANOWIRE_UTILITY_HPP
