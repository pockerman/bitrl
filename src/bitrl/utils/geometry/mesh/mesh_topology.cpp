#include "bitrl/utils/geometry/mesh/mesh_topology.h"
#include "bitrl/utils/geometry/mesh/element.h"
#include "bitrl/utils/geometry/mesh/face_element.h"
#include "bitrl/utils/geometry/mesh/node.h"

#include <exception>

namespace bitrl
{
namespace utils
{
namespace geom
{

template <int spacedim>
template <typename T, typename C>
T *MeshTopology<spacedim>::add_entity(T *t, C &c)
{

    if (!t)
    {
        throw std::invalid_argument("Cannot add nullptr to container");
    }

    // If the user requests a valid id that doesn't correspond to an
    // existing node, let's give them that id, resizing the nodes
    // container if necessary.

    if (!t->has_valid_id())
    {
        t->set_id(c.size());
    }

    uint_t id = t->get_id();

    if (id < c.size())
    {
    }
    else
    {

        c.resize(id + 1, nullptr);
    }

    c[id] = t;

    return t;
}

template <int spacedim> MeshTopology<spacedim>::~MeshTopology() { clear_topology(); }

template <int spacedim> Node<spacedim> *MeshTopology<spacedim>::add_node(Node<spacedim> *n)
{
    return add_entity(n, nodes_);
}

template <int spacedim> Element<spacedim> *MeshTopology<spacedim>::add_element(Element<spacedim> *e)
{
    return add_entity(e, elements_);
}

template <int spacedim> void MeshTopology<spacedim>::clear_topology_nodes()
{

    typedef typename std::vector<Node<spacedim> *>::iterator iterator;

    iterator it = nodes_.begin();
    iterator it_end = nodes_.end();

    for (; it != it_end; ++it)
    {
        delete *it;
    }

    nodes_.clear();
}

template <int spacedim> void MeshTopology<spacedim>::clear_topology_elements()
{

    typedef typename std::vector<Element<spacedim> *>::iterator iterator;

    iterator it = elements_.begin();
    iterator it_end = elements_.end();

    for (; it != it_end; ++it)
    {
        delete *it;
    }

    elements_.clear();
}

template <int spacedim> void MeshTopology<spacedim>::clear_topology()
{

    clear_topology_nodes();
    clear_topology_elements();
    clear_topology_edges();
    clear_topology_faces();
}

template <int dim> typename MeshTopology<dim>::face_ptr_t MeshTopology<dim>::face(uint_t i)
{
    // AssertLibSimPP(i<faces_.size(),ExecInvalidIndex(i,0,faces_.size()));
    return faces_[i];
}

template <> void MeshTopology<1>::clear_topology_edges() { clear_topology_nodes(); }

template <> void MeshTopology<1>::clear_topology_faces() { clear_topology_nodes(); }

template <> MeshTopology<1>::edge_ptr_t MeshTopology<1>::edge(uint_t i) { return nodes_[i]; }

template <> MeshTopology<1>::face_ptr_t MeshTopology<1>::face(uint_t i) { return nodes_[i]; }

template <> MeshTopology<1>::edge_ptr_t MeshTopology<1>::add_edge(MeshTopology<1>::edge_ptr_t edge)
{
    return add_entity(static_cast<Node<1> *>(edge), nodes_);
}

template <> MeshTopology<1>::face_ptr_t MeshTopology<1>::add_face(MeshTopology<1>::face_ptr_t face)
{
    return add_entity(static_cast<Node<1> *>(face), nodes_);
}

template <>
void MeshTopology<1>::faces(const MeshConnectivity &faces_idx,
                            std::vector<MeshTopology<1>::cface_ptr_t> &faces_ptr) const
{

    faces_ptr.resize(faces_idx.size(), nullptr);

    for (uint_t i = 0; i < faces_idx.size(); ++i)
    {

        faces_ptr[i] = static_cast<cface_ptr_t>(nodes_[faces_idx.connection_idx(i)]);
    }
}

template <> void MeshTopology<2>::clear_topology_edges()
{

    typedef std::vector<MeshTopology<2>::edge_ptr_t>::iterator iterator;

    iterator it = edges_.begin();
    iterator it_end = edges_.end();

    for (; it != it_end; ++it)
    {
        delete *it;
    }

    edges_.clear();
}

template <> void MeshTopology<2>::clear_topology_faces() { clear_topology_edges(); }

template <> MeshTopology<2>::edge_ptr_t MeshTopology<2>::edge(uint_t i) { return edges_[i]; }

template <> MeshTopology<2>::face_ptr_t MeshTopology<2>::face(uint_t i) { return edges_[i]; }

template <> MeshTopology<2>::edge_ptr_t MeshTopology<2>::add_edge(MeshTopology<2>::edge_ptr_t edge)
{
    return add_entity(edge, edges_);
}

template <> MeshTopology<2>::face_ptr_t MeshTopology<2>::add_face(MeshTopology<2>::face_ptr_t face)
{
    return add_entity(face, edges_);
}

template <>
void MeshTopology<2>::faces(const MeshConnectivity &faces_idx,
                            std::vector<MeshTopology<2>::cface_ptr_t> &faces_ptr) const
{

    faces_ptr.resize(faces_idx.size(), nullptr);

    for (uint_t i = 0; i < faces_idx.size(); ++i)
    {
        faces_ptr[i] = edges_[faces_idx.connection_idx(i)];
    }
}

template class MeshTopology<1>;
template class MeshTopology<2>;

} // namespace geom
} // namespace utils
} // namespace bitrl
