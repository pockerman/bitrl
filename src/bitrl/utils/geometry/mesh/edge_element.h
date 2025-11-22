#ifndef EDGE_ELEMENT_H
#define EDGE_ELEMENT_H

#include "bitrl/utils/geometry/mesh/element.h"
#include "bitrl/utils/geometry/mesh/face_element.h"

namespace bitrl{
namespace utils{
namespace geom{
template<int dim> class EdgeElem;

template<int dim>
class EdgeElem: public FaceElement<dim, 1>
{
public:

    typedef FaceElement<2, 1>::node_ptr_t node_ptr_t;

    /// \brief Constructor
    EdgeElem(uint_t id, uint_t proc_id=0);

    /// \brief Constructor
    EdgeElem(uint_t id, uint_t n_nodes, uint_t proc_id);

};

template<>
class EdgeElem<1>: public Element<1>
{
public:

    typedef  Element<1>::neighbor_ptr_t neighbor_ptr_t;
    typedef  Element<1>::neighbor_ref_t neighbor_ref_t;
    typedef  Element<1>::node_ptr_t node_ptr_t;
    typedef  Element<1>::edge_ptr_t edge_ptr_t;
    typedef  Element<1>::cedge_ptr_t cedge_ptr_t;
    typedef  Element<1>::edge_ref_t edge_ref_t;
    typedef  Element<1>::cedge_ref_t cedge_ref_t;
    typedef  Element<1>::face_ptr_t face_ptr_t;
    typedef  Element<1>::cface_ptr_t cface_ptr_t;
    typedef  Element<1>::face_ref_t face_ref_t;
    typedef  Element<1>::cface_ref_t cface_ref_t;

    /// \brief Constructor
    EdgeElem(uint_t id, uint_t proc_id=0);

    /// \brief How many nodes the element has
    virtual uint_t n_nodes()const override{return 2;}

    /// \brief Set the i-th node
    virtual void set_node(uint_t i, node_ptr_t node) override final;

    /// \brief Returns the i-th node
    virtual node_ptr_t get_node(uint_t n) override final;

    /// \brief How many vertices the element has
    virtual uint_t n_vertices()const override final{return 2;}

    /// \brief How many edges the element has
    virtual uint_t n_edges()const override final{return 2;}

    /// \brief How many faces the element has
    virtual uint_t n_faces()const override final{return 2;}

    /// \brief Resize the space for the faces
    /// an EdgeElem<1> does not have edges/faces
    /// these are Node type
    virtual void resize_faces()override final{}

    /// \brief Set the f-th face of the element
    /// An EdgeElem<1> does not have extra faces
    virtual void set_face(uint_t, face_ptr_t)override final{}

    /// \brief Returns the f-face
    virtual cface_ref_t get_face(uint_t f)const override;

    /// \brief Returns the f-face
    virtual face_ref_t get_face(uint_t f) override;

    /// \brief Returns the number of neighbors
    virtual uint_t n_neighbors()const{return 2;}

    /// \brief Returns the node ids of the vertices of the
    /// given face
    virtual void face_vertices(uint_t f, std::vector<uint_t>& ids)const override final;

    /// \brief Returns the volume of the element
    virtual real_t volume()const override final;

	///
    /// \brief Returns the local id relevant to the calling object
    /// of the  passed  object
    virtual uint_t which_face_am_i(cface_ref_t face)const override final{throw std::logic_error("Not implemented");}

    /// \brief Returns the face normal vector
    virtual const DynVec<real_t> face_normal_vector(uint_t f)const override final{throw std::logic_error("Not implemented");}

};

}
}
}
#endif // EDGE_ELEMENT_H
