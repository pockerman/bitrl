#include "bitrl/utils/geometry/mesh/mesh.h"
#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/geometry/mesh/element.h"
#include "bitrl/utils/geometry/mesh/element_builder.h"
#include "bitrl/utils/geometry/mesh/node.h"

namespace bitrl
{
namespace utils
{
namespace geom
{

template <int spacedim> Mesh<spacedim>::Mesh() : n_boundaries_(0), topology_() {}

template <int spacedim> Mesh<spacedim>::~Mesh() {}

template <int spacedim>
Node<spacedim> *Mesh<spacedim>::create_vertex(const GeomPoint<spacedim, real_t> &point,
                                              uint_t global_id, uint_t pid)
{

    Node<spacedim> *n = new Node<spacedim>(point, global_id, pid);
    n->make_vertex();

    topology_.add_node(n);

    return n;
}

template <int spacedim>
Node<spacedim> *Mesh<spacedim>::create_node(const GeomPoint<spacedim, real_t> &point,
                                            uint_t global_id, uint_t pid)
{

    Node<spacedim> *n = new Node<spacedim>(point, global_id, pid);
    topology_.add_node(n);
    return n;
}

template <int spacedim>
Element<spacedim> *Mesh<spacedim>::create_element(ElementType::sub_type t, uint_t global_id,
                                                  uint_t pid)
{

    Element<spacedim> *e = ElementBuilder<spacedim>::build_element(t);

    // e->set_mesh(*this);
    e->set_id(global_id);
    e->set_pid(pid);

    topology_.add_element(e);
    return e;
}

template <int spacedim>
typename Mesh<spacedim>::edge_ptr_t Mesh<spacedim>::create_edge(ElementType::sub_type t,
                                                                uint_t global_id, uint_t pid)
{

    typename Mesh<spacedim>::edge_ptr_t edge = EdgeBuilder<spacedim>::build_edge(t);

    edge->set_id(global_id);
    topology_.add_edge(edge);
    return edge;
}

template <int spacedim>
typename Mesh<spacedim>::face_ptr_t Mesh<spacedim>::create_face(ElementType::sub_type t,
                                                                uint_t global_id, uint_t pid)
{

    typename Mesh<spacedim>::face_ptr_t edge = FaceBuilder<spacedim>::build_face(t);
    edge->set_id(global_id);
    topology_.add_face(edge);
    return edge;
}

template <int spacedim> uint_t Mesh<spacedim>::n_vertices() const
{

    uint_t nvertices = std::count_if(topology_.nodes_begin(), topology_.nodes_end(),
                                     [](const Node<spacedim> *n) { return n->is_vertex(); });

    return nvertices;
}

template <int spacedim> uint_t Mesh<spacedim>::n_active_faces() const
{

    return std::count_if(topology_.faces_begin(), topology_.faces_end(),
                         [](typename Mesh<spacedim>::cface_ptr_t face)
                         { return face->is_active(); });
}

template class Mesh<1>;
template class Mesh<2>;

} // namespace geom
} // namespace utils
} // namespace bitrl
