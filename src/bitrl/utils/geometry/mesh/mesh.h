#ifndef MESH_H
#define MESH_H

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/utils/filtered_iterator.h"
#include "bitrl/utils/geometry/mesh/element_type.h"
#include "bitrl/utils/geometry/mesh/mesh_topology.h"

#include "boost/noncopyable.hpp"

namespace bitrl
{
namespace utils
{
namespace geom
{

// forward declarations
template <int dim, typename T> class GeomPoint;

///
/// \brief A class that represents a mesh.
///
template <int spacedim> class Mesh : private boost::noncopyable
{

  public:
    typedef typename MeshTopology<spacedim>::edge_ptr_t edge_ptr_t;
    typedef typename MeshTopology<spacedim>::face_ptr_t face_ptr_t;
    typedef typename MeshTopology<spacedim>::cedge_ptr_t cedge_ptr_t;
    typedef typename MeshTopology<spacedim>::cface_ptr_t cface_ptr_t;

    /// \brief Node iteration
    typedef typename std::vector<Node<spacedim> *>::iterator node_iterator_impl;
    typedef typename std::vector<Node<spacedim> *>::const_iterator cnode_iterator_impl;

    /// \brief Element iteration
    typedef typename std::vector<Element<spacedim> *>::iterator element_iterator_impl;
    typedef typename std::vector<Element<spacedim> *>::const_iterator celement_iterator_impl;

    /// \brief Edge iteration
    typedef typename std::vector<edge_ptr_t>::iterator edge_iterator_impl;
    typedef typename std::vector<edge_ptr_t>::const_iterator cedge_iterator_impl;

    /// \brief Face iteration
    typedef typename std::vector<face_ptr_t>::iterator face_iterator_impl;
    typedef typename std::vector<face_ptr_t>::const_iterator cface_iterator_impl;

    const static int dimension = spacedim;

    /// \brief Constructor. Creates an empty Mesh
    Mesh();

    /// \brief dtor
    virtual ~Mesh();

    void reserve_n_nodes(uint_t n) { topology_.reserve_n_nodes(n); }
    void reserve_n_elements(uint_t n) { topology_.reserve_n_elements(n); }
    void reserve_n_edges(uint_t n) { topology_.reserve_n_edges(n); }
    void reserve_n_faces(uint_t n) { topology_.reserve_n_faces(n); }

    /**
     *\detailed create a vertex by passing in the coordinates
     *of the vertex. This will create a Node so optionally we can
     *pass in the global id and the processor id of the created node
     */
    Node<spacedim> *create_vertex(const GeomPoint<spacedim, real_t> &point,
                                  uint_t global_id = bitrl::consts::INVALID_ID, uint_t pid = 0);

    /**
     *\detailed create a Node
     */
    Node<spacedim> *create_node(const GeomPoint<spacedim, real_t> &point,
                                uint_t global_id = bitrl::consts::INVALID_ID, uint_t pid = 0);

    /**
     *\detailed create an Element and get back the pointer
     */
    Element<spacedim> *create_element(ElementType::sub_type t,
                                      uint_t global_id = bitrl::consts::INVALID_ID, uint_t pid = 0);

    /**
     *\detailed create an edge and get back the pointer
     */
    edge_ptr_t create_edge(ElementType::sub_type t, uint_t global_id = bitrl::consts::INVALID_ID,
                           uint_t pid = 0);

    /**
     *\detailed create a face and get back the pointer
     */
    face_ptr_t create_face(ElementType::sub_type t, uint_t global_id = bitrl::consts::INVALID_ID,
                           uint_t pid = 0);

    /**
     * \detailed set the number of boundaries for the mesh
     */
    void set_n_boundaries(uint_t nb) { n_boundaries_ = nb; }

    /**
     *\detailed read/write acces to the n-th node
     */
    Node<spacedim> *node(uint_t n) { return topology_.node(n); }

    /**
     *\detailed read acces to the n-th node
     */
    const Node<spacedim> *node(uint_t n) const { return topology_.node(n); }

    /**
     *\detailed read/write access to the n-th element
     */
    Element<spacedim> *element(uint_t e) { return topology_.element(e); }

    /**
     *\detailed read access to the n-th element
     */
    const Element<spacedim> *element(uint_t e) const { return topology_.element(e); }

    /**
     *\detailed how many vertices the mesh has
     */
    uint_t n_vertices() const;

    /**
     *\detailed how many nodes the mesh has
     */
    uint_t n_nodes() const { return topology_.n_nodes(); }

    /**
     *\detailed how many elements the mesh has
     */
    uint_t n_elements() const { return topology_.n_elements(); }

    /// \brief How many edges the mesh has
    uint_t n_edges() const;

    /// \brief how many faces the mesh has
    uint_t n_faces() const;

    /**
     * get the active faces of the mesh
     */
    uint_t n_active_faces() const;

    /// \brief Return read/write access to the topology of the mesh
    MeshTopology<spacedim> *topology() { return &topology_; }

    /// \brief Read access to the topology of the mesh
    const MeshTopology<spacedim> *topology() const { return &topology_; }

    /// \brief Get the pointers to the faces described by the given MeshConnectivity
    void faces(const MeshConnectivity &faces_idx, std::vector<cface_ptr_t> &faces_ptr) const
    {
        topology_.faces(faces_idx, faces_ptr);
    }

    /// \brief Get the number of boundaries of the mesh
    uint_t n_boundaries() const { return n_boundaries_; }

    /// \brief Raw node iteration
    node_iterator_impl nodes_begin() { return topology_.nodes_begin(); }
    node_iterator_impl nodes_end() { return topology_.nodes_end(); }

    /// \brief Raw node iteration
    cnode_iterator_impl nodes_begin() const { return topology_.nodes_begin(); }
    cnode_iterator_impl nodes_end() const { return topology_.nodes_end(); }

    /// \brief Raw elements iteration
    element_iterator_impl elements_begin() { return topology_.elements_begin(); }
    element_iterator_impl elements_end() { return topology_.elements_end(); }

    /// \brief Raw elements iteration
    celement_iterator_impl elements_begin() const { return topology_.elements_begin(); }
    celement_iterator_impl elements_end() const { return topology_.elements_end(); }

    /// \brief Raw faces iteration
    edge_iterator_impl edges_begin() { return topology_.edges_begin(); }
    edge_iterator_impl edges_end() { return topology_.edges_end(); }

    /// \brief Raw faces iteration
    cedge_iterator_impl edges_begin() const { return topology_.edges_begin(); }
    cedge_iterator_impl edges_end() const { return topology_.edges_end(); }

    /// \brief Raw faces iteration
    face_iterator_impl faces_begin() { return topology_.faces_begin(); }
    face_iterator_impl faces_end() { return topology_.faces_end(); }

    /// \brief Raw faces iteration
    cface_iterator_impl faces_begin() const { return topology_.faces_begin(); }
    cface_iterator_impl faces_end() const { return topology_.faces_end(); }

  private:
    /// \brief The number of boundaries of the mesh
    uint_t n_boundaries_;

    /// \brief Describes how the various mesh entities are connected
    MeshTopology<spacedim> topology_;
};

template <int spacedim> inline uint_t Mesh<spacedim>::n_edges() const
{
    return topology_.n_edges();
}

template <int spacedim> inline uint_t Mesh<spacedim>::n_faces() const
{
    return topology_.n_faces();
}

template <> inline uint_t Mesh<1>::n_edges() const { return n_vertices(); }

template <> inline uint_t Mesh<1>::n_faces() const { return n_edges(); }

} // namespace geom
} // namespace utils
} // namespace bitrl

#endif // MESH_H
