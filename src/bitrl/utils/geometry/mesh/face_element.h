#ifndef FACE_ELEMENT_H
#define FACE_ELEMENT_H

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/geometry/mesh/mesh_entity.h"
#include "bitrl/utils/geometry/mesh/edge_face_selector.h"

#include <array>
#include <any>

namespace bitrl{
namespace utils{
namespace geom{

///forward declarations
//template<int spacedim> class Mesh;

template<int dim> class Element;
template<int dim> class Node;
template<int dim,int topodim> class FaceElement;


/// \detailed A FaceElement<spacedim,0> represents a point
/// in spacedim spatial dimensions. Edges of spatial dimension
/// \p spacedim return FaceElement<spacedim,0> ptrs when asked
/// about their edges and faces. We do not derive from Element<spacedim,topodim>
/// because we want to have this class to behave like a Point and not like an Element.
/// There is a specialization for 1D that provides some extended functionality like
/// inquiring if the point is on the boundary and on which boundary, which Element
/// owns the point and if it is an internal point who is the neighbor
template<int spacedim>
class FaceElement<spacedim, 0>: public GeomPoint<spacedim>,
                                public MeshEntity
{

public:

	///
	/// \brief 
	///
    FaceElement();

	///
	/// \brief 
	///
    explicit FaceElement(uint_t global_id,
                         real_t val=0.0,
                         uint_t pid=0,
						 const std::any& data=std::any());

	///
	/// \brief
	///
    FaceElement(const std::array<real_t, spacedim>& coords,
                uint_t global_id, 
				uint_t pid=0,
				const std::any& data=std::any());

	///
	/// \brief
	///
    FaceElement(const GeomPoint<spacedim>& point,
                uint_t global_id,
                uint_t pid,
				const std::any& data);

	///
	/// \brief
	///
    FaceElement(const FaceElement& t);

	///
	/// \brief
	///
    FaceElement& operator=(const FaceElement& t);
  
    ///
	/// \brief
	///
    virtual ~FaceElement()
    {}

    /// \brief detailed make this node a vertex
    void make_vertex(){is_vertex_ = true;}

    /// \brief  returns true iff \p make_vertex has been called on this object
    bool is_vertex()const{return is_vertex_;}
    
	///
    /// \brief
	///
    real_t volume()const{return 0.0;}

    /// \brief The centroid is the point itself
    GeomPoint<spacedim> centroid()const{return *this;}

    bool on_boundary () const {return boundary_indicator_!=bitrl::consts::INVALID_ID;}
    uint_t boundary_indicator()const{return boundary_indicator_;}

private:
  
  bool is_vertex_;
  uint_t boundary_indicator_;
  
};


///
/// FaceElement<spacedim,1> exists only when MeshGeometry::dim_ = 2 or 3
/// These inherit from Element<spacedim,1> as they are elements embedded in
/// a space of MeshGeometry::dim_ space. FaceElement<2,1> objects actually bound the
/// 2D elements we support. They can be inquired about the boundary they may be located
/// which element owns them and if they are internal which element shares them.
/// In 2D a FaceElement<2,1> is owned/shared by a Face<2> object meaning an Element<2,2>.
/// In 3D a FaceElement<3,1> is owned/shared by a Face<3> object meaning an Element<3,2>.
/// Concrete classes implement the pure virtual functions inherited by Element<spacedim,1>
/// Concrete classes of this class are: Edge<2> and Edge<3>.
///
template<>
class FaceElement<2,1>
{

public:
 
    static const int dim_ = 2;
    static const int topodim_ = 1;

    typedef Node<2>* node_ptr_t;
    typedef const Node<2>* cnode_ptr_t;
    typedef Node<2>& node_ref_t;
    typedef const Node<2>& cnode_ref_t;
    typedef EdgeSelector<2>::ptr_t  edge_ptr_t;
    typedef EdgeSelector<2>::cptr_t cedge_ptr_t;
    typedef EdgeSelector<2>::ptr_t  face_ptr_t;
    typedef EdgeSelector<2>::cptr_t cface_ptr_t;

    /// \brief dtor
    virtual ~FaceElement(){}

    /// \brief Resize the number of nodes the face has
    virtual void resize_nodes();

    /// \brief How many nodes the element has
    virtual uint_t n_nodes()const {return n_nodes_;}

    /// \brief Set the i-th node
    virtual void set_node(uint_t i, node_ptr_t node);

    /// \brief Returns the i-th node
    virtual node_ptr_t node_ptr(uint_t n);

    /// \brief How many vertices the element has
    virtual uint_t n_vertices()const {return 2;}

    /// \brief How many edges the element has
    virtual uint_t n_edges()const {return 2;}

    /// \brief How many faces the element has
    virtual uint_t n_faces()const {return 2;}

    /// set the pointer to the
    /// element that holds the side.
    void set_owner_element(Element<2>* o){owner_ = o;}

    /// set the pointer to the element that
    /// shares this side. It also sets the old sharing element
    /// to the element we use before calling this function
    void set_shared_element(Element<2>* n){neighbor_ = n;}

    /// set the pointers for the shared and owner of this side
    void set_owner_shared_elements(Element<2>* o, Element<2>* n)
    {owner_ = o; neighbor_ = n;}

    /// \brief set the boundary indicator for the FaceElement
    void set_boundary_indicator(uint_t bind){boundary_indicator_=bind;}

    /// \detailed read/write access to the owner of this
    /// face. May return NULL
    Element<2>* owner(){return owner_;}
 
    /// \detailed read/write access to the neighbor of this
    /// face may return NULL in this case the face is on the
    /// boundary
    Element<2>* neighbor(){return neighbor_;}

    /// \brief Returns the vertices ids
    virtual std::vector<uint_t> get_vertices_ids()const;

    /// Returns \p true iff a boundary indicator has been set
    bool on_boundary () const {return boundary_indicator_ != bitrl::consts::INVALID_ID;}
 
    /// Get the boundary indicator of the face
    uint_t boundary_indicator()const{return boundary_indicator_;}

    /// \brief Calculate the volume of the element
    real_t volume()const;

    /// \brief computes a reference distance between the
    /// centroids of the two elements sharing the FaceElement
    /// If this lies on the the boundary then the centroid of
    /// the FaceElementis used
    real_t owner_neighbor_distance()const;

    /// Print the information for the MeshEntity
    virtual std::ostream& print_mesh_entity_info(std::ostream &out)const;
 
    /// \brief Returns the centroid of the face element
    const GeomPoint<2> centroid()const;

    /// \brief Returns true if the owner of the cell has
    /// the given id
    bool is_owner(uint_t id)const;

    /// \brief Read/write access to the neighbor element
    Element<2>& get_neighbor();

    /// \brief Read reference to the neighbor of the side
    const Element<2>& get_neighbor()const;

    /// \brief Read/write access to the neighbor element
    Element<2>& get_owner();

    /// \brief Read reference to the neighbor of the side
    const Element<2>& get_owner()const;

    /// \brief Returns the v-th vertex
    cnode_ref_t get_vertex(uint_t v)const;

    /// \brief Returns the v-th vertex
    node_ref_t get_vertex(uint_t v);

    /// \brief Returns the normal vector to the face
    const DynVec<real_t> normal_vector()const;

    bool is_active()const{return true;}
    uint_t get_id()const{return id_;}
    void set_id(uint_t id){id_ = id;}
    bool has_valid_id()const{return true;}

protected:

    FaceElement();
    FaceElement(uint_t id,uint_t pid=0);
    FaceElement(uint_t id, uint_t nnodes, uint_t pid);

    /// \brief The nodes of the element
    std::vector<node_ptr_t> nodes_;
 
private:
 
    /// \brief The owner of the FaceElement
    Element<2>* owner_;

    /// \brief The neighbor of the FaceElement
    Element<2>* neighbor_;

    /// \detailed the boundary indicator of the edge
    /// the default is KernelConsts::invalid_size_type()
    uint_t boundary_indicator_;

    /// ink with the print_mesh_entity_info
    std::ostream& print_(std::ostream& o)const;

    /// \brief The global id of the face
    uint_t id_;

    /// \brief Number of nodes by default is two
    uint_t n_nodes_;

};


inline
std::ostream& 
FaceElement<2,1>::print_mesh_entity_info(std::ostream &out)const{
  return print_(out);
}

/*

//forward declaration 
class Face;

template<>
class FaceElement<3,1>
{

 protected:
 

  FaceElement();
  FaceElement(uint_t id,uint_t pid=0);
  FaceElement(uint_t id, uint_t nnodes, uint_t pid);
  
 public:
 
 static const int dim_ = 3;
 static const int topodim_ = 1;
 
 typedef EdgeSelector<3>::ptr_t   edge_ptr_t;
 typedef EdgeSelector<3>::ptr_t face_ptr_t;
 
     virtual ~FaceElement(){}


     void set_owner_element(Face* o){owner_ = o;}
     void set_shared_element(Face* n){neighbor_ = n;}
     void set_owner_shared_elements(Face* o, Face* n)
     {owner_ = o; neighbor_ = n;}
      void set_boundary_indicator(uint_t bind){boundary_indicator_=bind;}
     Face* owner(){return owner_;}
     Face* neighbor(){return neighbor_;}
     bool on_boundary () const {return true;}
     uint_t boundary_indicator()const{return boundary_indicator_;}



      virtual std::ostream& print_mesh_entity_info(std::ostream &out)const;

     /// \brief
     real_t volume()const{return 0.0;}

     /// \brief Returns the centroid of the face element
     GeomPoint<3> centroid()const{return GeomPoint<3>(0.0);}

     /// \brief Returns true if the owner of the cell has
     /// the given id
     bool is_owner(uint_t id)const{return  true;}

     /// \brief Read/write access to the neighbor element
     Element<3>& get_neighbor(){}

     /// \brief Read reference to the neighbor of the side
     const Element<3>& get_neighbor()const{}

     /// \brief Read/write access to the neighbor element
     Element<3>& get_owner(){}

     /// \brief Read reference to the neighbor of the side
     const Element<3>& get_owner()const{}

       bool is_active()const{return true;}

       uint_t get_id()const{return id_;}
       void set_id(uint_t id){id_ = id;}
       bool has_valid_id()const{return true;}

private:
 
  Face* owner_;
  Face* neighbor_;
  uint_t boundary_indicator_;

  std::ostream& print_(std::ostream& o)const;

  uint_t id_;


};

inline
std::ostream& 
FaceElement<3,1>::print_mesh_entity_info(std::ostream &out)const
{
  return print_(out);
}

*/

//
///**
//  * A Face<3> IsA FaceElement<3,2>
//  *
//  * This is the base class for representing faces in MeshGeometry::dim_ = 3
//  * spatial dimension. Concrete classes of this class implement the pure virtual
//  * functions inherited by Element<3,2> class. Concrete classes of this class are
//  * Quad<3>, Tri<3>.
//  */
//  
//template<>
//class FaceElement<3,2>
//{
//
// protected:
// 
//
// FaceElement();
// 
// FaceElement(uint_t id,uint_t pid=0);
//
// public:
// 
// 
// static const int dim_ = 3;
// static const int topodim_ = 2;
// 
// typedef EdgeSelector<3>::ptr_t edge_ptr_t;
// typedef EdgeSelector<3>::ptr_t face_ptr_t;
// 
// 
// /**
//   *\detailed dtor
//   */
// virtual ~FaceElement(){}
// 
//
// /**
//   *  set the pointer to the
//   *  element that holds the side.
//   */
// void set_owner_element(Element<3>* o){owner_ = o;}
//
//
// /**
//   * set the pointer to the element that
//   * shares this side. It also sets the old sharing element
//   * to the element we use before calling this function
//   */
// void set_shared_element(Element<3>* n){neighbor_ = n;}
//
//
// /**
//   * set the pointers for the shared and owner of this side
//   */
// void set_owner_shared_element(Element<3>* o, Element<3>* n)
// {owner_ = o; neighbor_ = n;}
// 
// 
// /**
//    *\detailed set the boundary indicator for the FaceElement
//    */
//  void set_boundary_indicator(uint_t bind){boundary_indicator_=bind;}
//  
//  
//
//
//  /**
//   *\detailed read/write access to the owner of this
//   * face. May return NULL
//   */
//  Element<3>* owner(){return owner_;}
//  
//  
//   /**
//   *\detailed read/write access to the neighbor of this
//   * face. May return NULL
//   */
//  Element<3>* neighbor(){return neighbor_;}
//
//
//  /**
//      * @returns \p true iff a boundary indicator has been set
//      */
//    bool on_boundary () const { return false;/*return boundary_indicator_!=LibSimPP::internal_mesh_entity_id();*/}
//  
//  
// /**
//   *\detailed get the boundary indicator of the face
//   */
// uint_t boundary_indicator()const{return boundary_indicator_;}
//  
//  
//  /**
//   *\detailed print the information for the MeshEntity
//   */
//  virtual std::ostream& print_mesh_entity_info(std::ostream &out)const;
//  
//    /// \brief
//    real_t volume()const{return 0.0;}
//
//    /// \brief Returns the centroid of the face element
//    /// \brief The centroid is the point itself
//    GeomPoint<3> centroid()const{return GeomPoint<3>(0.0);}
//
//    /// \brief computes a reference distance between the
//    /// centroids of the two elements sharing the FaceElement
//    /// If this lies on the the boundary then the centroid of
//    /// the FaceElementis used
//    real_t owner_neighbor_distance()const{return 0.0;}
//  
//   bool is_active()const{return true;}
//
//   /// \brief Returns true if the owner of the cell has
//   /// the given id
//   bool is_owner(uint_t id)const{throw std::logic_error("Not implemented");}
//
//   /// \brief Read/write access to the neighbor element
//   Element<3>& get_neighbor(){throw std::logic_error("Not implemented");}
//
//   /// \brief Read reference to the neighbor of the side
//   const Element<3>& get_neighbor()const{throw std::logic_error("Not implemented");}
//
//
//   /// \brief Read/write access to the neighbor element
//   Element<3>& get_owner(){throw std::logic_error("Not implemented");}
//
//   /// \brief Read reference to the neighbor of the side
//   const Element<3>& get_owner()const{throw std::logic_error("Not implemented");}
//
//   /// \brief Returns the normal vector to the face
//   const DynVec<real_t> normal_vector()const{throw std::logic_error("Not implemented");}
//
//   uint_t get_id()const{return id_;}
//   void set_id(uint_t id){id_ = id;}
//   bool has_valid_id()const{return true;}
//
//  protected:
//  
//  
//  /**
//   *\detailed the owner of the FaceElement
//   */
//  Element<3>* owner_;
// 
// 
// /**
//   *\detailed the neighbor of the FaceElement
//   */
//  Element<3>* neighbor_;
//  
//  
//  /**
//    *\detailed the boundary indicator of the edge
//    */
//  uint_t boundary_indicator_;
//
//  uint_t id_;
//  
//  /**
//    *\detailed we fail to link with the print_mesh_entity_info
//    */
//  std::ostream& print_(std::ostream& o)const;
//
//};
//
//inline
//std::ostream& 
//FaceElement<3,2>::print_mesh_entity_info(std::ostream &out)const
//{
//  return print_(out);
//}

}
}
}
#endif
