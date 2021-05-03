//
// Created by acsr on 4/30/21.
//

#ifndef NANOWIREPLANNER_PLANNER_BUILDER_HPP
#define NANOWIREPLANNER_PLANNER_BUILDER_HPP

#include "nanowire_sst.hpp"
#include "nanowire_ref_isst.hpp"
#include "nanowire_isst.hpp"
#include "nanowire_planner.hpp"
#include "nanowire_system.hpp"

namespace acsr {

    struct PlannerBuilder {
    public:
        PlannerBuilder() = default;

        ~PlannerBuilder() = default;

        PlannerBuilder(const PlannerBuilder &) = delete;

        PlannerBuilder &operator=(const PlannerBuilder) = delete;

        /***
         * create an planner pointer
         * @param planner_type planner type
         * @param dynamic_system the dynamic system used for this planner
         * @return an shared pointer to planner
         */
        static std::shared_ptr<Planner> create(PlannerType planner_type, std::shared_ptr<NanowireSystem> dynamic_system) {
            std::shared_ptr<Planner> planner;
            switch (planner_type) {

                case PlannerType::e_SST:
                    planner = std::make_shared<SST>(dynamic_system);
                    break;
                case PlannerType::e_iSST:
                    planner = std::make_shared<iSST>(dynamic_system);
                    break;
                case PlannerType::e_Ref_iSST:
                    planner = std::make_shared<RefSST>(dynamic_system);
            }
            return planner;
        }
    };
}



#endif //NANOWIREPLANNER_PLANNER_BUILDER_HPP
