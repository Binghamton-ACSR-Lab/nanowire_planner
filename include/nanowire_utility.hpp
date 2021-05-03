//
// Created by acsr on 4/29/21.
//

#ifndef NANOWIREPLANNER_NANOWIRE_UTILITY_HPP
#define NANOWIREPLANNER_NANOWIRE_UTILITY_HPP

#include <random>

namespace acsr{

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

    double randomDouble(int _min, int _max) {
        std::uniform_real_distribution<double> distribution(_min,_max);
        return distribution(_random_engine);
    }

    /***
     * set a random seed for random engin
     */
    void setRandomSeed(unsigned seed) {
        _random_engine.seed(seed);
    }

    int getHeuristic(int n_wires,const ElectordePositionType& state1,const ElectordePositionType& state2){
        int value = 0;
        for(auto i=0;i<n_wires;++i) {
            auto temp_v=(state1[i]-state2[i]).lpNorm<1>();
            value = std::max(value, temp_v);
        }
        return value;
    }




}



#endif //NANOWIREPLANNER_NANOWIRE_UTILITY_HPP
