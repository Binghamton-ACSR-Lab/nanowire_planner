//
// Created by acsr on 5/25/21.
//

#ifndef BOOST_RTREE_EIGEN_ADAPTOR_HPP
#define BOOST_RTREE_EIGEN_ADAPTOR_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <acado/acado_toolkit.hpp>
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


namespace boost::geometry::traits{

    template <typename CoordinateType, int DimensionCount>
    struct tag<Eigen::Matrix<CoordinateType, DimensionCount, 1>>{
        using type = point_tag;
    };

    template <typename CoordinateType, int DimensionCount>
    struct coordinate_type<Eigen::Matrix<CoordinateType, DimensionCount, 1>>{
        typedef CoordinateType type;
    };

    template <typename CoordinateType, int DimensionCount>
    struct dimension<Eigen::Matrix<CoordinateType, DimensionCount, 1>>: boost::mpl::int_<DimensionCount> {

    };

    template <typename CoordinateType, int DimensionCount, size_t Dimension>
    struct access<Eigen::Matrix<CoordinateType, DimensionCount, 1>, Dimension>{
        static inline CoordinateType get(Eigen::Matrix<CoordinateType, DimensionCount, 1> const& a)
        {
            return a(Dimension);
        }

        static inline void set(Eigen::Matrix<CoordinateType, DimensionCount, 1>& a,
                               CoordinateType const& value){
            a(Dimension) = value;
        }
    };
}

#define BOOST_GEOMETRY_REGISTER_EIGEN_MATRIX_CS(CoordinateSystem) \
namespace boost::geometry::traits{                     \
template <class T, int N>                              \
struct coordinate_system<Eigen::Matrix<T, N, 1>>               \
{                                                              \
  typedef CoordinateSystem type;                               \
};                                                             \
}

#endif //BOOST_RTREE_EIGEN_ADAPTOR_HPP
