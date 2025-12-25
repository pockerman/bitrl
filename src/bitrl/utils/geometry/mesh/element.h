#ifndef ELEMENT_H
#define ELEMENT_H

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/geometry/mesh/edge_face_selector.h"
#include "bitrl/utils/geometry/mesh/element_traits.h"
#include "bitrl/utils/geometry/mesh/mesh_entity.h"

#include <any>

#include <functional>
#include <map>
#include <string>

namespace bitrl
{
namespace utils
{
namespace geom
{

template <int dim> class Node;

///
/// \brief Wraps the notion of an element
///
template <int dim> class Element : public MeshEntity
{

  public:
    typedef Element<dim> *neighbor_ptr_t;
    typedef Element<dim> &neighbor_ref_t;
    typedef const Element<dim> &cneighbor_ref_t;
    typedef Node<dim> *node_ptr_t;
    typedef typename element_traits<Element<dim>>::edge_ptr_t edge_ptr_t;
    typedef typename element_traits<Element<dim>>::cedge_ptr_t cedge_ptr_t;
    typedef typename element_traits<Element<dim>>::edge_ref_t edge_ref_t;
    typedef typename element_traits<Element<dim>>::cedge_ref_t cedge_ref_t;
    typedef typename element_traits<Element<dim>>::face_ptr_t face_ptr_t;
    typedef typename element_traits<Element<dim>>::cface_ptr_t cface_ptr_t;
    typedef typename element_traits<Element<dim>>::face_ref_t face_ref_t;
    typedef typename element_traits<Element<dim>>::cface_ref_t cface_ref_t;

    /// \brief Destructor
    virtual ~Element();

    /// \brief How many vertices the element has
    virtual uint_t n_vertices() const = 0;

    /// \brief How many nodes the element has
    virtual uint_t n_nodes() const = 0;

    /// \brief Set the i-th node
    virtual void set_node(uint_t i, node_ptr_t node);

    /// \brief Append a  node to the nodes list
    virtual void append_node(node_ptr_t node);

    /// \brief Reserve space for nodes
    virtual void resize_nodes();

    /// \brief Returns the i-th node
    virtual node_ptr_t get_node(uint_t n);

    /// \brief How many edges the element has
    virtual uint_t n_edges() const = 0;

    /// \brief How many faces the element has
    virtual uint_t n_faces() const = 0;

    /// \brief Resize the space for the faces
    virtual void resize_faces() = 0;

    /// \brief Returns the f-face
    virtual cface_ref_t get_face(uint_t f) const = 0;

    /// \brief Returns the f-face
    virtual face_ref_t get_face(uint_t f) = 0;

    /// \brief Set the f-th face of the element
    virtual void set_face(uint_t f, face_ptr_t face) = 0;

    /// \brief Set the i-th neighbor
    virtual void set_neighbor(uint_t n, neighbor_ptr_t neigh);

    /// \brief Returns the number of neighbors
    virtual uint_t n_neighbors() const = 0;

    /// \brief Reserve space for neighbors
    virtual void resize_neighbors();

    /// \brief Access the n-th neighbor
    virtual cneighbor_ref_t get_neighbor(uint_t n) const;

    /// \brief Access the n-th neighbor pointer
    virtual neighbor_ptr_t neighbor_ptr(uint_t n);

    /// \brief Access the n-th neighbor pointer
    virtual const neighbor_ptr_t neighbor_ptr(uint_t n) const;

    /// \brief Returns the node ids of the vertices of the
    /// given face
    virtual void face_vertices(uint_t f, std::vector<uint_t> &ids) const = 0;

    /// \brief Returns the volume of the element
    virtual real_t volume() const = 0;

    /// \brief Returns the average location of the
    /// vertices of the element
    virtual GeomPoint<dim> centroid() const;

    /// \brief Returns the local id relevant to the calling object
    /// of the  passed  object
    virtual uint_t which_face_am_i(cface_ref_t face) const = 0;

    /// \brief Returns the face normal vector
    virtual const DynVec<real_t> face_normal_vector(uint_t f) const = 0;

    /// \brief Returns the local id relevant to the calling object
    /// of the  passed  object
    uint_t which_neighbor_am_i(const Element<dim> &element) const;

    /// \brief Return the vertices of the element
    std::vector<node_ptr_t> get_vertices() const;

  protected:
    /// \brief Constructor
    Element();

    /// \brief Constructor
    Element(uint_t id, uint_t pid);

    /// \brief Constructor
    Element(uint_t id, uint_t pid, const std::any &data);

    /// \brief The neighbors of the element
    std::vector<neighbor_ptr_t> neginbors_;

    /// \brief The nodes of the element
    std::vector<node_ptr_t> nodes_;
};

} // namespace geom

} // namespace utils
} // namespace bitrl

#endif // ELEMENT_H
