#include "bitrl/utils/geometry/mesh/quad_element.h"
#include "bitrl/utils/geometry/mesh/face_element.h"
#include "bitrl/utils/geometry/mesh/node.h"
//#include "kernel/geometry/geometry_utils.h"

#include <cmath>
#include <exception>
namespace bitrl
{
namespace utils
{
namespace geom
{

namespace
{

void do_get_face_vertices(uint_t f, const std::vector<Node<2> *> &nodes, std::vector<uint_t> &ids)
{

    if (ids.size() != 2)
    {
        ids.clear();
        ids.resize(2);
    }

    switch (f)
    {
    case 0:
        ids[0] = nodes[0]->get_id();
        ids[1] = nodes[1]->get_id();
        break;
    case 1:
        ids[0] = nodes[1]->get_id();
        ids[1] = nodes[2]->get_id();
        break;
    case 2:
        ids[0] = nodes[2]->get_id();
        ids[1] = nodes[3]->get_id();
        break;
    case 3:
        ids[0] = nodes[3]->get_id();
        ids[1] = nodes[0]->get_id();
        break;
    }
}

GeomPoint<2> cross_product(const GeomPoint<2> &p1, const GeomPoint<2> &p2) {}

} // namespace

Quad<2>::Quad(uint_t n) : Element<2>(), n_nodes_per_edge_(n) {}

Quad<2>::Quad(uint_t id, uint_t pid, const std::any &data, uint_t n)
    : Element<2>(id, pid, data), n_nodes_per_edge_(n)
{
}

void Quad<2>::resize_faces() { faces_.resize(n_faces(), nullptr); }

void Quad<2>::set_face(uint_t f, Quad<2>::face_ptr_t face)
{

    if (f >= n_faces())
    {
        throw std::logic_error("Invalid node index: " + std::to_string(f) + " not in [0," +
                               std::to_string(n_faces()) + ")");
    }

    faces_[f] = face;
}

Quad<2>::cface_ref_t Quad<2>::get_face(uint_t f) const
{

    if (f >= n_faces())
    {
        throw std::logic_error("Invalid face index: " + std::to_string(f) + " not in [0," +
                               std::to_string(n_faces()) + ")");
    }

    if (faces_.empty())
    {
        throw std::logic_error("Faces have not been initialized for element: " +
                               std::to_string(this->get_id()));
    }

    if (faces_[f] == nullptr)
    {
        throw std::logic_error("Face requested is NULL");
    }

    return *faces_[f];
}

Quad<2>::face_ref_t Quad<2>::get_face(uint_t f)
{

    if (f >= n_faces())
    {
        throw std::logic_error("Invalid node index: " + std::to_string(f) + " not in [0," +
                               std::to_string(n_faces()) + ")");
    }

    if (faces_.empty())
    {
        throw std::logic_error("Faces have not been initialized for element: " +
                               std::to_string(this->get_id()));
    }

    if (faces_[f] == nullptr)
    {
        throw std::logic_error("Face requested is NULL");
    }

    return *faces_[f];
}

void Quad<2>::face_vertices(uint_t f, std::vector<uint_t> &ids) const
{

    if (f >= n_faces())
    {
        throw std::logic_error("Invalid node index: " + std::to_string(f) + " not in [0," +
                               std::to_string(n_faces()) + ")");
    }

    if (ids.size() != 2)
    {
        ids.clear();
        ids.resize(2);
    }

    // prefer going via the faces list
    // but sometime this may not be available
    if (this->faces_.empty())
    {
        do_get_face_vertices(f, this->nodes_, ids);
    }
    else
    {

        if (this->faces_[f] == nullptr)
        {
            do_get_face_vertices(f, this->nodes_, ids);
        }
        else
        {
            ids = this->faces_[f]->get_vertices_ids();
        }
    }
}

real_t Quad<2>::volume() const
{

    //        3           2    D           C
    // QUAD4: o-----------o	 o-----------o
    //        |           |	 |           |
    //        |           |	 |           |
    //        |           | 	 |           |
    //        |           |	 |           |
    //        |           |	 |           |
    //        o-----------o	 o-----------o
    //        0           1	 A           B

    // Vector pointing from A to C
    GeomPoint<2> AC(*this->nodes_[2] - *this->nodes_[0]);

    // Vector pointing from A to B
    GeomPoint<2> AB(*this->nodes_[1] - *this->nodes_[0]);

    // Vector pointing from A to D
    GeomPoint<2> AD(*this->nodes_[3] - *this->nodes_[0]);

    // The diagonal vector minus the side vectors
    GeomPoint<2> AC_AB_AD(AC - AB - AD);

    // Check for quick return for planar QUAD4.  This will
    // be the most common case, occuring for all purely 2D meshes.
    if (AC_AB_AD == GeomPoint<2>(0.0))
    {
        return cross_product(AB, AD).L2_norm();
    }
    else
    {
        // Use 2x2 quadrature to approximate the surface area.  (The
        // true integral is too difficult to compute analytically.)  The
        // accuracy here is exactly the same as would be obtained via a
        // call to Elem::volume(), however it is a bit more optimized to
        // do it this way.  The technique used is to integrate the magnitude
        // of the normal vector over the whole area.  See for example,
        //
        // Y. Zhang, C. Bajaj, G. Xu. Surface Smoothing and Quality
        // Improvement of Quadrilateral/Hexahedral Meshes with Geometric
        // Flow. The special issue of the Journal Communications in
        // Numerical Methods in Engineering (CNME), submitted as an
        // invited paper, 2006.
        // http://www.ices.utexas.edu/~jessica/paper/quadhexgf/quadhex_geomflow_CNM.pdf

        // 4-point rule
        const static real_t q[2] = {0.5 - std::sqrt(3.) / 6., 0.5 + std::sqrt(3.) / 6.};

        real_t vol = 0.;
        for (uint_t i = 0; i < 2; ++i)
            for (uint_t j = 0; j < 2; ++j)
                vol += cross_product((AB + q[i] * AC_AB_AD), (AD + q[j] * AC_AB_AD)).L2_norm();

        return 0.25 * vol;
    }
}

/// \brief Returns the local id relevant to the calling object
/// of the  passed  object
uint_t Quad<2>::which_face_am_i(Quad<2>::cface_ref_t face) const
{

    auto foreign_vertices = face.get_vertices_ids();
    for (uint_t f = 0; f < n_faces(); ++f)
    {

        auto &localface = *faces_[f];
        auto vertices = localface.get_vertices_ids();

        if (vertices.empty() || foreign_vertices.empty())
        {
            throw std::logic_error("Empty vertices list found");
        }

        if (vertices.size() != foreign_vertices.size())
        {
            throw std::logic_error("Invalid face type comparison requested");
        }

        if (vertices[0] == foreign_vertices[0] && vertices[1] == foreign_vertices[1])
        {
            return f;
        }
    }

    return bitrl::consts::INVALID_ID;
}

const DynVec<real_t> Quad<2>::face_normal_vector(uint_t f) const
{

    switch (f)
    {

    case 0: // bottom side of the quadrilateral
    {
        const GeomPoint<2> n0 = *this->nodes_[0];
        const GeomPoint<2> n1 = *this->nodes_[1];
        DynVec<real_t> values; // = {n1[1]-n0[1],n0[0]-n1[0]};
        return values;         //{n1[1]-n0[1],n0[0]-n1[0]}; //value; DynVec<real_t>(values);
    }
    case 1: /// right side of the quadrilateral
    {
        const GeomPoint<2> n1 = *this->nodes_[1];
        const GeomPoint<2> n2 = *this->nodes_[2];
        auto values = {n2[1] - n1[1], n1[0] - n2[0]};
        // return {n2[1]-n1[1],n1[0]-n2[0]}; //DynVec<real_t>(values);
    }
    case 2: /// top side of the quadrilateral
    {
        const GeomPoint<2> n2 = *this->nodes_[2];
        const GeomPoint<2> n3 = *this->nodes_[3];
        auto values = {n3[1] - n2[1], n2[0] - n3[0]};
        // return {n3[1]-n2[1],n2[0]-n3[0]}; //DynVec<real_t>(values);
    }
    case 3: /// left side of the quadrilateral
    {
        const GeomPoint<2> n0 = *this->nodes_[0];
        const GeomPoint<2> n3 = *this->nodes_[3];
        auto values = {n0[1] - n3[1], n3[0] - n0[0]};
        // return {n0[1]-n3[1],n3[0]-n0[0]}; //DynVec<real_t>(values);
    }
    default:
    {
        throw std::logic_error("Invalid face index: " + std::to_string(f) + "not in [0," +
                               std::to_string(n_faces()) + ")");
    }
    }

    // return {0.0, 0.0}; //DynVec<real_t>(2, 0.0);
}

} // namespace geom
} // namespace utils
} // namespace bitrl
