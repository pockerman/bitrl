#include "bitrl/utils/geometry/mesh/edge_element.h"
#include "bitrl/utils/geometry/mesh/node.h"
#include <exception>

namespace bitrl
{
namespace utils
{
namespace geom
{

EdgeElem<1>::EdgeElem(uint_t id, uint_t pid) : Element<1>(id, pid) {}

void EdgeElem<1>::set_node(uint_t n, EdgeElem<1>::node_ptr_t node)
{

    if (n >= n_nodes())
    {
        throw std::logic_error("Invalid node index " + std::to_string(n) + " not in [0, " +
                               std::to_string(n_nodes()) + ")");
    }

    this->nodes_[n] = node;
}

EdgeElem<1>::node_ptr_t EdgeElem<1>::get_node(uint_t n)
{

    if (n >= n_nodes())
    {
        throw std::logic_error("Invalid node index " + std::to_string(n) + " not in [0, " +
                               std::to_string(n_nodes()) + ")");
    }

    if (this->nodes_.empty())
    {
        throw std::logic_error("Nodes list have not been initialized");
    }

    return this->nodes_[n];
}

EdgeElem<1>::cface_ref_t EdgeElem<1>::get_face(uint_t f) const {}

EdgeElem<1>::face_ref_t EdgeElem<1>::get_face(uint_t f) {}

void EdgeElem<1>::face_vertices(uint_t f, std::vector<uint_t> &ids) const
{

    ids.clear();
    ids.resize(1);

    ids[0] = f == 0 ? this->nodes_[0]->get_id() : this->nodes_[1]->get_id();
}

real_t EdgeElem<1>::volume() const
{

    if (this->nodes_.empty())
    {
        throw std::logic_error("Nodes list have not been initialized");
    }

    return this->nodes_[0]->distance(*this->nodes_[1]);
}

template <int dim> EdgeElem<dim>::EdgeElem(uint_t id, uint_t pid) : FaceElement<dim, 1>(id, pid) {}

template <int dim>
EdgeElem<dim>::EdgeElem(uint_t id, uint_t n_nodes, uint_t pid)
    : FaceElement<dim, 1>(id, n_nodes, pid)
{
}

template class EdgeElem<2>;

} // namespace geom
} // namespace utils
} // namespace bitrl