//
// Created by acsr on 5/27/21.
//

#ifndef BOOST_RTREE_NODE_ADAPTOR_HPP
#define BOOST_RTREE_NODE_ADAPTOR_HPP


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <vector>
#include <eigen3/Eigen/Core>
#include "tree_node.hpp"
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


namespace boost::geometry::traits{

    //BOOST_GEOMETRY_DETAIL_SPECIALIZE_POINT_TRAITS(cpPtr, 2, double, cs::cartesian);

    template <typename CoordinateType, int DimensionCount>
    struct tag<std::shared_ptr<acsr::Node<CoordinateType,DimensionCount>>>{
        using type = point_tag;
    };

    template <typename CoordinateType, int DimensionCount>
    struct coordinate_type<std::shared_ptr<acsr::Node<CoordinateType,DimensionCount>>>{
        typedef CoordinateType type;
    };

    template <typename CoordinateType, int DimensionCount>
    struct dimension<std::shared_ptr<acsr::Node<CoordinateType,DimensionCount>>>: boost::mpl::int_<DimensionCount> {

    };


    template <typename CoordinateType, int DimensionCount, size_t Dimension>
    struct access<std::shared_ptr<acsr::Node<CoordinateType,DimensionCount>>, Dimension>{
        static_assert(0 <= Dimension && Dimension < DimensionCount, "Out of range");
        //using Point = std::shared_ptr<acsr::Node<CoordinateType,DimensionCount>>;
        //using CoordinateType = typename coordinate_type<Point>::type;

        static inline CoordinateType get(std::shared_ptr<acsr::Node<CoordinateType,DimensionCount>> const& a)
        {
            return (*a)[Dimension];
        }

        static inline void set(std::shared_ptr<acsr::Node<CoordinateType,DimensionCount>>& a,
                               CoordinateType const& value){
            (*a)[Dimension] = value;
        }
    };
}

#define BOOST_GEOMETRY_REGISTER_NODE_CS(CoordinateSystem) \
namespace boost::geometry::traits{                     \
template <class T, int N>                              \
struct coordinate_system<std::shared_ptr<acsr::Node<T,N>>>               \
{                                                              \
  typedef CoordinateSystem type;                               \
};                                                             \
}






#endif //BOOST_RTREE_NODE_ADAPTOR_HPP
