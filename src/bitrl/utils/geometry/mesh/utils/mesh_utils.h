#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include "kernel/geometry/shapes/circle.h"
#include "kernel/geometry/geometry_utils.h"
#include "kernel/discretization/element_mesh_iterator.h"
#include "kernel/discretization/mesh_predicates.h"

#include <vector>
#include <tuple>
#include <limits>

namespace kernel {
namespace discretization {
namespace utils {

template<typename ElementTp>
const kernel::GeomPoint<ElementTp::dimension>
find_point_on_element_closest_to(const ElementTp& element, const GeomPoint<ElementTp::dimension>& p,
                                 uint_t nsamples, real_t tol){

    static_assert (ElementTp::dimension == 2, "Element dimension is not 2");
    typedef kernel::GeomPoint<ElementTp::dimension> point_t;

    std::vector<point_t> points_on_segment(nsamples + 2);

    auto v0 = element.get_vertex(0);
    auto v1 = element.get_vertex(1);

    real_t distance = v0.distance(v1);

    ///...plus 2 because we also account for the end poinst
    real_t h = distance/(real_t) (nsamples + 2);

    /// the first point is the staring vertex
    points_on_segment[0] = v0;

    /// the first point is the staring vertex
    points_on_segment[0] = v0;

    if(std::fabs(v1[0] - v0[0]) < tol){
        /// we have line that is perpendicular to the
        /// x-axis and we need to act differently to avoid infinity

        for(uint_t sp = 1; sp <=nsamples; ++sp){

            real_t xi = v0[0];
            points_on_segment[sp] = point_t({xi, v0[0] + sp*h});
        }
    }
    else if(std::fabs(v1[1] - v0[1]) < tol){
        /// we have line that is parallel to the
        /// x-axis and we need to act differently to avoid infinity

        for(uint_t sp = 1; sp <=nsamples; ++sp){

            real_t xi = v0[1];
            points_on_segment[sp] = point_t({v0[0] + sp*h, xi});
        }
    }
    else{

        /// calculate the coefficients of the line
        /// connecting the two vertices
        real_t alpha = (v1[1] - v0[1])/(v1[0] - v0[0]);
        real_t beta = (v0[1]*v1[0] + v0[0]*v1[1])/(v1[0] - v0[0]);

        for(uint_t sp = 1; sp <=nsamples; ++sp){

            real_t xi = v0[0] + sp*h;
            points_on_segment[sp] = point_t({xi, alpha*xi + beta});
        }
    }
    /// add the ending vertex of the line segment
    points_on_segment[nsamples + 1] = v1;

    auto min_point = get_point_with_min_distance(p, points_on_segment);
    return min_point;
}


template<typename ElementTp>
real_t distance_from(const ElementTp& element,
                     const GeomPoint<ElementTp::dimension>& p,
                     uint_t nsamples, real_t tol){


    auto point = find_point_on_element_closest_to(element, p, nsamples, tol);
    /*typedef GeomPoint<ElementTp::dimension> point_t;

    std::vector<point_t> points_on_segment(nsamples + 2);

    auto v0 = element.get_vertex(0);
    auto v1 = element.get_vertex(1);

    real_t distance = v0.distance(v1);

    ///...plus 2 because we also account for the end poinst
    real_t h = distance/(real_t) (nsamples + 2);

    /// the first point is the staring vertex
    points_on_segment[0] = v0;

    if(std::fabs(v1[0] - v0[0]) < tol){
        /// we have line that is perpendicular to the
        /// x-axis and we need to act differently to avoid infinity

        for(uint_t sp = 1; sp <=nsamples; ++sp){

            real_t xi = v0[0];
            points_on_segment[sp] = point_t({xi, v0[0] + sp*h});
        }
    }
    else if(std::fabs(v1[1] - v0[1]) < tol){
        /// we have line that is parallel to the
        /// x-axis and we need to act differently to avoid infinity

        for(uint_t sp = 1; sp <=nsamples; ++sp){

            real_t xi = v0[1];
            points_on_segment[sp] = point_t({v0[0] + sp*h, xi});
        }

    }
    else{

        /// calculate the coefficients of the line
        /// connecting the two vertices
        real_t alpha = (v1[1] - v0[1])/(v1[0] - v0[0]);
        real_t beta = (v0[1]*v1[0] + v0[0]*v1[1])/(v1[0] - v0[0]);

        for(uint_t sp = 1; sp <=nsamples; ++sp){

            real_t xi = v0[0] + sp*h;
            points_on_segment[sp] = point_t({xi, alpha*xi + beta});
        }
    }
    /// add the ending vertex of the line segment
    points_on_segment[nsamples + 1] = v1;

    auto min_point = get_point_with_min_distance(p, points_on_segment);*/
    return  point.distance(p);
}


/// \brief Find the element in the mesh
/// with the smallest distance from the point
template<typename MeshTp>
const typename MeshTp::element_t*
find_closest_element(const MeshTp& mesh, const GeomPoint<MeshTp::dimension>& p,
                     uint_t nsamples, real_t tol){

    numerics::ConstElementMeshIterator<numerics::Active, MeshTp> filter(mesh);
    typedef typename MeshTp::element_t element_t;
    element_t* ptr = nullptr;
    real_t dist = std::numeric_limits<real_t>::max();

    auto begin = filter.begin();
    auto end   = filter.end();

    for(; begin != end; ++begin){

        auto element = *begin;
        auto dist_cur = distance_from(*element, p, nsamples, tol);

        if(dist_cur < dist){
            dist = dist_cur;
            ptr = element;
        }
    }

    return ptr;
}




template<typename MeshTp>
std::tuple<const typename MeshTp::point_t, const typename MeshTp::element_t*>
find_closest_point_to(const MeshTp& mesh, const GeomPoint<2>& p,
                      uint_t nsamples, real_t tol){

    static_assert (MeshTp::dimension == 2, "Mesh dimension is not 2");

    typedef typename MeshTp::point_t point_t;

    //std::vector<point_t> points_on_segment(nsamples + 2);
    std::vector<point_t> min_points;
    min_points.reserve(mesh.n_elements());

    numerics::ConstElementMeshIterator<numerics::Active, MeshTp> filter(mesh);

    auto begin = filter.begin();
    auto end   = filter.end();

    for(; begin != end; ++begin){

        auto element = *begin;

        /*auto v0 = element->get_vertex(0);
        auto v1 = element->get_vertex(1);

        real_t distance = v0.distance(v1);

        ///...plus 2 because we also account for the end poinst
        real_t h = distance/(real_t) (nsamples + 2);

        /// the first point is the staring vertex
        points_on_segment[0] = v0;

        if(std::fabs(v1[0] - v0[0]) < tol){
            /// we have line that is perpendicular to the
            /// x-axis and we need to act differently to avoid infinity

            for(uint_t sp = 1; sp <=nsamples; ++sp){

                real_t xi = v0[0];
                points_on_segment[sp] = point_t({xi, v0[0] + sp*h});
            }
        }
        else if(std::fabs(v1[1] - v0[1]) < tol){
            /// we have line that is parallel to the
            /// x-axis and we need to act differently to avoid infinity

            for(uint_t sp = 1; sp <=nsamples; ++sp){

                real_t xi = v0[1];
                points_on_segment[sp] = point_t({v0[0] + sp*h, xi});
            }
        }
        else{

            /// calculate the coefficients of the line
            /// connecting the two vertices
            real_t alpha = (v1[1] - v0[1])/(v1[0] - v0[0]);
            real_t beta = (v0[1]*v1[0] + v0[0]*v1[1])/(v1[0] - v0[0]);

            for(uint_t sp = 1; sp <=nsamples; ++sp){

                real_t xi = v0[0] + sp*h;
                points_on_segment[sp] = point_t({xi, alpha*xi + beta});
            }
        }
        /// add the ending vertex of the line segment
        points_on_segment[nsamples + 1] = v1;

        /// loop over the points and find the point with the minimum distance
        auto min_point = get_point_with_min_distance(p, points_on_segment);*/
        auto min_point = find_point_on_element_closest_to(*element, p, nsamples, tol);
        min_points.push_back(min_point);

    }

    auto result = get_point_with_min_distance(p, min_points);

    /// find out to which segment this point belongs to
    auto element = find_closest_element(mesh, result, nsamples, tol);

    return {result, element};
}

/// \brief Returns the intersection points of the
/// Circle with the elements of the LineMesh.
/// The algorithn implemented is the one described here
/// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899%E2%80%8B
template<typename MeshTp>
const std::vector<typename MeshTp::point_t> find_intersections(const MeshTp& mesh,
                                                                 const Circle& circle){

    static_assert (MeshTp::dimension == 2, "Mesh dimension is not 2");

    std::vector<typename MeshTp::point_t> intersections;

    const auto r = circle.radius();
    const auto center = circle.center();

    numerics::ConstElementMeshIterator<numerics::Active, MeshTp> filter(mesh);
    auto begin = filter.begin();
    auto end   = filter.end();

    for(; begin != end; ++begin){

        auto element = *begin;
        auto v0 = element->get_vertex(0);
        auto v1 = element->get_vertex(1);

        auto d = v1 - v0;
        auto f = v0 - center;

        auto a = d.dot(d);
        auto b = 2.0*f.dot(d);
        auto c = f.dot(f) - r*r;

        auto discriminant = b*b-4*a*c;

        auto intersects = has_intersection(discriminant, b, a);
        if(intersects.first){

            /// thats an intersection point
            auto intersection = v0  + d*intersects.second;
            intersections.push_back(intersection);
        }
    }

    return intersections;
}

}

}

}

#endif // MESH_UTILS_H
