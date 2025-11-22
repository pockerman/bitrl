#ifndef QUAD_ELEMENT_H
#define QUAD_ELEMENT_H

#include "bitrl/bitrl_types.h"
#include "bitrl/utils/geometry/mesh/element.h"

#include <any>

namespace bitrl{
namespace utils{
namespace geom{

/// \detailed a class that represents quad type elements in dim spatial dimension.
/// In 2D this object acts like a proper element. In 3D this class
/// has extented functionality making act like a face to Hex elements
///
/// We use the following counter-clockwise numbering scheme

///        3           2     D           C
/// QUAD4: o-----------o	 o-----------o
///        |           |	 |           |
///        |           |	 |           |
///        |           | 	 |           |
///        |           |	 |           |
///        |           |	 |           |
///        o-----------o	 o-----------o
///        0           1	 A           B

template<int dim> class Quad;


template<>
class Quad<2>: public Element<2>
{

 public:

    typedef typename Element<2>::node_ptr_t     node_ptr_t;
    typedef typename Element<2>::neighbor_ptr_t neighbor_ptr_t;
    typedef typename Element<2>::neighbor_ref_t neighbor_ref_t;
    typedef typename Element<2>::edge_ptr_t     edge_ptr_t;
    typedef typename Element<2>::cedge_ptr_t    cedge_ptr_t;
    typedef typename Element<2>::edge_ref_t     edge_ref_t;
    typedef typename Element<2>::cedge_ref_t    cedge_ref_t;
    typedef typename Element<2>::face_ptr_t     face_ptr_t;
    typedef typename Element<2>::cface_ptr_t    cface_ptr_t;
    typedef typename Element<2>::face_ref_t     face_ref_t;
    typedef typename Element<2>::cface_ref_t    cface_ref_t;
 
 
    /// \brief ctor
    explicit Quad(uint_t nnodes=4);
   
    /// \brief Constructor
    explicit Quad(uint_t id, uint_t pid, const std::any& data, uint_t n=4);
 
    /// \brief Returns the total number of nodes for the element
    virtual uint_t n_nodes()const override final{return n_nodes_per_edge_;}
   
    /// \brief A quad has 4 vertices independent of the dimension
    virtual uint_t n_vertices()const override final{return 4;}

    /// \brief A quad has 4 neighbors independent of the dimension
    virtual uint_t n_neighbors()const override final{return 4;}

    /// \brief Resize the space for the faces
    virtual void resize_faces();
   
    /// \brief Set the f-th face of the element
    virtual void set_face(uint_t f, face_ptr_t face)override final;

    /// \brief A quad has 4 sides independent of the dimension
    virtual uint_t n_faces()const  final{return 4;}

    /// \brief Returns the f-face
    virtual cface_ref_t get_face(uint_t f)const override;

    /// \brief Returns the f-face
    virtual face_ref_t get_face(uint_t f) override;
   
    /// \brief A quad has 4 edges independent of the dimension
    virtual uint_t n_edges()const  final{return 4;}

    /// \brief Returns the node ids of the vertices of the
    /// given face
    virtual void face_vertices(uint_t f, std::vector<uint_t>& ids)const override;
   
    /// \brief Returns the volume of the element
    virtual real_t volume()const override final;

    /// \brief Returns the local id relevant to the calling object
    /// of the  passed  object
    virtual uint_t which_face_am_i(cface_ref_t face)const override final;

    /// \brief Returns the face normal vector
    virtual const DynVec<real_t> face_normal_vector(uint_t f)const override final;

   
private:
 
  
  ///  
  /// @brief the number of nodes
  /// per edge the quad has
  ///
  uint_t n_nodes_per_edge_;
	
  /// \brief The faces of the quad
  std::vector<face_ptr_t> faces_;

};


}
}
}
#endif
