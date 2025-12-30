#include "kernel/discretization/utils/line_mesh_utils.h"
#include "kernel/discretization/edge_element.h"
#include "kernel/discretization/element_mesh_iterator.h"
#include "kernel/discretization/line_mesh.h"
#include "kernel/discretization/mesh_predicates.h"
#include "kernel/discretization/node.h"
#include "kernel/geometry/geometry_utils.h"
#include "kernel/geometry/shapes/circle.h"

#include <cmath>

namespace kernel
{
namespace numerics
{

const GeomPoint<2> find_closest_point_to(const LineMesh<2> &mesh, const GeomPoint<2> &p,
                                         uint_t nsamples, real_t tol)
{

    std::vector<GeomPoint<2>> points_on_segment(nsamples + 2);
    std::vector<GeomPoint<2>> min_points;
    min_points.reserve(mesh.n_elements());

    ConstElementMeshIterator<Active, LineMesh<2>> filter(mesh);
    auto begin = filter.begin();
    auto end = filter.end();

    for (; begin != end; ++begin)
    {

        auto element = *begin;

        auto v0 = element->get_vertex(0);
        auto v1 = element->get_vertex(1);

        real_t distance = v0.distance(v1);

        ///...plus 2 because we also account for the end poinst
        real_t h = distance / (real_t)(nsamples + 2);

        /// the first point is the staring vertex
        points_on_segment[0] = v0;

        if (std::fabs(v1[0] - v0[0]) < tol)
        {
            /// we have line that is perpendicular to the
            /// x-axis and we need to act differently to avoid infinity

            for (uint_t sp = 1; sp <= nsamples; ++sp)
            {

                real_t xi = v0[0];
                points_on_segment[sp] = GeomPoint<2>({xi, v0[0] + sp * h});
            }
        }
        else
        {

            /// calculate the coefficients of the line
            /// connecting the two vertices
            real_t alpha = (v1[1] - v0[1]) / (v1[0] - v0[0]);
            real_t beta = (v0[1] * v1[0] + v0[0] * v1[1]) / (v1[0] - v0[0]);

            for (uint_t sp = 1; sp <= nsamples; ++sp)
            {

                real_t xi = v0[0] + sp * h;
                points_on_segment[sp] = GeomPoint<2>({xi, alpha * xi + beta});
            }
        }
        /// add the ending vertex of the line segment
        points_on_segment[nsamples + 1] = v1;

        /// loop over the points and find the point with the minimum distance
        auto min_point = get_point_with_min_distance(p, points_on_segment);
        min_points.push_back(min_point);
    }

    auto result = get_point_with_min_distance(p, min_points);
    return result;
}

namespace
{

std::pair<bool, real_t> has_intersection(real_t discriminant, real_t b, real_t a)
{

    std::pair<bool, real_t> result = {false, 0.0};

    /// no intersection
    if (discriminant < 0.)
        return result;

    /// ray didn't totally miss sphere,
    /// so there is a solution to
    /// the equation.

    discriminant = std::sqrt(discriminant);

    /// either solution may be on or off the ray so need to test both
    /// t1 is always the smaller value, because BOTH discriminant and
    /// a are nonnegative.
    auto t1 = (-b - discriminant) / (2 * a);
    auto t2 = (-b + discriminant) / (2 * a);

    /// 3x HIT cases:
    ///          -o->             --|-->  |            |  --|->
    /// Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

    /// 3x MISS cases:
    ///       ->  o                     o ->              | -> |
    /// FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

    if (t1 >= 0 && t1 <= 1)
    {

        /// t1 is the intersection, and it's closer than t2
        /// (since t1 uses -b - discriminant)
        /// Impale, Poke
        return {true, t1};
    }

    /// here t1 didn't intersect so we are either started
    /// inside the sphere or completely past it
    if (t2 >= 0 && t2 <= 1)
    {
        /// ExitWound
        return {true, t2};
    }

    /// no intn: FallShort, Past, CompletelyInside
    return result;
}

} // namespace

const std::vector<GeomPoint<2>> find_intersections(const LineMesh<2> &mesh, const Circle &circle)
{

    std::vector<GeomPoint<2>> intersections;

    const auto r = circle.radius();
    const auto center = circle.center();

    ConstElementMeshIterator<Active, LineMesh<2>> filter(mesh);
    auto begin = filter.begin();
    auto end = filter.end();

    for (; begin != end; ++begin)
    {

        auto element = *begin;
        auto v0 = element->get_vertex(0);
        auto v1 = element->get_vertex(1);

        auto d = v1 - v0;
        auto f = v0 - center;

        auto a = d.dot(d);
        auto b = 2.0 * f.dot(d);
        auto c = f.dot(f) - r * r;

        auto discriminant = b * b - 4 * a * c;

        auto intersects = has_intersection(discriminant, b, a);
        if (intersects.first)
        {

            /// thats an intersection point
            auto intersection = v0 + d * intersects.second;
            intersections.push_back(intersection);
        }
    }

    return intersections;
}

} // namespace numerics
} // namespace kernel
