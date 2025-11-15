#ifndef LINE_MESH_UTILS_H
#define LINE_MESH_UTILS_H

#include "kernel/geometry/geom_point.h"

#include <vector>

namespace kernel {

/// forward declarations
class Circle;

namespace numerics {

template<int dim> class LineMesh;


/// \brief Find the closest point on the given LineMesh
/// The function loops over the elements in the
/// mesh and uses samples the EdgeElement using the given
/// number of sampling points plus two as it accounts for the
/// vertices as well. From the calculated points
/// it picks the one with the smallest distance
const GeomPoint<2> find_closest_point_to(const LineMesh<2>& mesh,
                                         const GeomPoint<2>&p, uint_t nsamples,
                                         real_t tol);

/// \brief Returns the intersection points of the
/// Circle with the elements of the LineMesh.
/// The algorithn implemented is the one described here
/// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899%E2%80%8B
const std::vector<GeomPoint<2>> find_intersections(const LineMesh<2>& mesh,
                                                   const Circle& circle);

}

}

#endif // LINE_MESH_UTILS_H
