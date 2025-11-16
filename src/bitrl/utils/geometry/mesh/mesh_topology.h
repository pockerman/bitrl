#ifndef MESH_TOPOLOGY_H
#define MESH_TOPOLOGY_H

#include "bitrl/bitrl_types.h"
#include "bitrl/utils/geometry/mesh/mesh_connectivity.h"
#include "bitrl/utils/geometry/mesh/element_traits.h"

#include <vector>

namespace bitrl{
namespace utils{
namespace geom{
	
	
  template<int spacedim> class Element;
  template<int spacedim,int topodim> class FaceElement;
  template<int dim> class Node;

 /** \class MeshTopology
  * MeshTopology stores the topology of a mesh, consisting of mesh
  * entities and connectivity (incidence relations for the mesh
  * entities). Note that the mesh entities don't need to be stored,
  * only the number of entities and the connectivity.
  *
  * A mesh entity e may be identified globally as a pair e = (dim,
  * i), where dim is the topological dimension and i is the index of
  * the entity within that topological dimension.
  *
  * As a minimum we assume that we know the nodes of the elements
  * in the mesh
  */
  
template<int spacedim>
class MeshTopology
{

public:

    /// \brief Constructor
    MeshTopology();

    /// \brief Constructor
    ~MeshTopology();

    typedef std::vector<MeshConnectivity>::iterator element_connectivity_iterator;

    typedef typename element_traits<Element<spacedim> >::edge_ptr_t edge_ptr_t;
    typedef typename element_traits<Element<spacedim> >::face_ptr_t face_ptr_t;

    typedef typename element_traits<Element<spacedim> >::cedge_ptr_t cedge_ptr_t;
    typedef typename element_traits<Element<spacedim> >::cface_ptr_t cface_ptr_t;

    typedef typename std::vector<Node<spacedim>* >::iterator node_iterator;
    typedef typename std::vector<Node<spacedim>* >::const_iterator const_node_iterator;

    typedef typename std::vector<Element<spacedim>* >::iterator element_iterator;
    typedef typename std::vector<Element<spacedim>* >::const_iterator const_element_iterator;

    typedef typename std::vector<edge_ptr_t>::iterator edge_iterator;
    typedef typename std::vector<edge_ptr_t>::const_iterator const_edge_iterator;

    typedef typename std::vector<face_ptr_t>::iterator face_iterator;
    typedef typename std::vector<face_ptr_t>::const_iterator const_face_iterator;

    /**
       *\brief reserve memory for n nodes
       */
     void reserve_n_nodes(uint_t n){nodes_.reserve(n);}

     /**
       *\brief reserve memory for n elements
       */
     void reserve_n_elements(uint_t n){elements_.reserve(n);}

      /**
       *\brief reserve memory for n edges
       */
     void reserve_n_edges(uint_t n){edges_.reserve(n);}

     /**
       *\brief reserve memory for n edges
       */
     void reserve_n_faces(uint_t n);

    /**
      *\brief add a new node to the mesh and get back the pointer
      */
    Node<spacedim>* add_node(Node<spacedim>* n);

    /**
      *\brief add a new element in the mesh and get back the pointer
      */
    Element<spacedim>* add_element(Element<spacedim>* e);

    /**
      *\brief add a new edge in the mesh and get back the pointer
      */
    edge_ptr_t add_edge(edge_ptr_t edge);

    /**
      *\brief add a new face in the mesh and get back the pointer
      */
    face_ptr_t add_face(face_ptr_t face);

    /**
      *\brief delete the nodes of the topology
      */
    void clear_topology_nodes();

    /**
      *\brief delete the elements of the topology
      */
    void clear_topology_elements();

    /**
      *\brief delete the edges of the topology
      */
    void clear_topology_edges();

    /**
      *\brief delete the faces of the topology
      */
    void clear_topology_faces();

    /**
      *\brief clear the topology
      */
    void clear_topology();
    
    /**
      *\brief get the number of nodes in the mesh
      */
    uint_t n_nodes()const{return nodes_.size();}
    
    /**
      *\brief get the number of elements in the mesh
      */
    uint_t n_elements()const{return elements_.size();}
    
    /**
      *\brief get the number of edges in the mesh
      */
    uint_t n_edges()const{return edges_.size();}


    /**
      *\brief get the number of edges in the mesh
      */
    uint_t n_faces()const;
    
    /**
      *\brief read/write access to the i-th node of the mesh
      */
    Node<spacedim>* node(uint_t i);
    
    /**
      *\brief read access to the i-th node of the mesh
      */
    const Node<spacedim>* node(uint_t i)const;
    
    
    /**
      *\brief read/write access to the i-th element of the mesh
      */
    Element<spacedim>* element(uint_t i);
    
    /**
      *\brief read access to the i-th element of the mesh
      */
    const Element<spacedim>* element(uint_t i)const;
    
    
    
    /**
      *\detailed read/write access to the i-th edge of the mesh
      */
    edge_ptr_t edge(uint_t i);
    
    
    /**
      *\detailed read/write access to the i-th face of the mesh
      */
    face_ptr_t face(uint_t f);
    
    
    /**
      *\detailed get the faces described by the given indices
      */
    void faces(const MeshConnectivity& faces_idx,std::vector<cface_ptr_t>& faces_ptr)const;
    
        
    node_iterator nodes_begin(){return nodes_.begin();}
    node_iterator nodes_end(){return   nodes_.end();}

    const_node_iterator nodes_begin()const{return nodes_.begin();}
    const_node_iterator nodes_end()const{return nodes_.end();}


    element_iterator elements_begin(){return elements_.begin();}
    element_iterator elements_end(){return   elements_.end();}

    const_element_iterator elements_begin()const{return elements_.begin();}
    const_element_iterator elements_end()const{return elements_.end();}


    edge_iterator edges_begin();
    edge_iterator edges_end();

    const_edge_iterator edges_begin()const;
    const_edge_iterator edges_end()const;


    face_iterator faces_begin();
    face_iterator faces_end();

    const_face_iterator faces_begin()const;
    const_face_iterator faces_end()const;
    
    
private:
    
    std::vector<Node<spacedim>* > nodes_;
    std::vector<edge_ptr_t> edges_;
    std::vector<face_ptr_t> faces_;
    std::vector<Element<spacedim>* > elements_;
    
    /**
      *\detailed add to the container C the entity T
      *and get back the pointer
      */
    template<typename T,typename C>
    T* add_entity(T* t, C& c);
  
};
  
  
template<int spacedim>
inline
MeshTopology<spacedim>::MeshTopology()
                       :
                       nodes_(),
                       elements_(),
                       edges_(),
                       faces_()
                       {}

template<int spacedim>
inline
Node<spacedim>*
MeshTopology<spacedim>::node(uint_t i){

 if(i >= nodes_.size()){
     throw std::logic_error("Invalid  index: "+std::to_string(i) +
                            " not in [0, "+
                            std::to_string(nodes_.size()));
 }
 
 return nodes_[i];
}


template<int spacedim>
inline
const Node<spacedim>*
MeshTopology<spacedim>::node(uint_t i)const{

    if(i >= nodes_.size()){
        throw std::logic_error("Invalid  index: "+std::to_string(i) +
                               " not in [0, "+
                               std::to_string(nodes_.size()));
    }
 
 return nodes_[i];
}



template<int spacedim>
inline
Element<spacedim>*
MeshTopology<spacedim>::element(uint_t i){

    if(i >= nodes_.size()){
        throw std::logic_error("Invalid  index: "+std::to_string(i) +
                               " not in [0, "+
                               std::to_string(elements_.size()));
    }
 
 return elements_[i];
}


template<int spacedim>
inline
const Element<spacedim>*
MeshTopology<spacedim>::element(uint_t i)const{

    if(i >= nodes_.size()){
        throw std::logic_error("Invalid  index: "+std::to_string(i) +
                               " not in [0, "+
                               std::to_string(elements_.size()));
    }

 return elements_[i];
}

template<int spacedim>
inline
typename MeshTopology<spacedim>::edge_iterator
MeshTopology<spacedim>::edges_begin(){
  return edges_.begin();
}

template<int spacedim>
inline
typename MeshTopology<spacedim>::edge_iterator
MeshTopology<spacedim>::edges_end(){
  return edges_.end();
}


template<int spacedim>
inline
typename MeshTopology<spacedim>::const_edge_iterator
MeshTopology<spacedim>::edges_begin()const{
  return edges_.begin();
}

template<int spacedim>
inline
typename MeshTopology<spacedim>::const_edge_iterator
MeshTopology<spacedim>::edges_end()const{
  return edges_.end();
}

template<int spacedim>
inline
typename MeshTopology<spacedim>::face_iterator 
MeshTopology<spacedim>::faces_begin(){
  return faces_.begin();
}

template<>
inline
MeshTopology<2>::face_iterator 
MeshTopology<2>::faces_begin(){
  return edges_.begin();
}

  
template<int spacedim>
inline
typename MeshTopology<spacedim>::face_iterator 
MeshTopology<spacedim>::faces_end(){
  return faces_.end();
}

template<>
inline
MeshTopology<2>::face_iterator 
MeshTopology<2>::faces_end(){
  return edges_.end();
}
  
template<int spacedim>
inline
typename MeshTopology<spacedim>::const_face_iterator 
MeshTopology<spacedim>::faces_begin()const{
  return faces_.begin();
}

template<>
inline
MeshTopology<2>::const_face_iterator 
MeshTopology<2>::faces_begin()const{
  return edges_.begin();
}
  
template<int spacedim>
inline
typename MeshTopology<spacedim>::const_face_iterator 
MeshTopology<spacedim>::faces_end()const{
 return  faces_.end();
}

template<>
inline
MeshTopology<2>::const_face_iterator 
MeshTopology<2>::faces_end()const{
  return edges_.end();
}

template<int spacedim>
inline
void 
MeshTopology<spacedim>::reserve_n_faces(uint_t n){
  faces_.reserve(n);
}


template<>
inline
void 
MeshTopology<2>::reserve_n_faces(uint_t n){
  edges_.reserve(n);
}

template<int spacedim>
inline
uint_t
MeshTopology<spacedim>::n_faces()const{
return faces_.size();
}

template<>
inline
uint_t
MeshTopology<2>::n_faces()const{
return edges_.size();
}

}
}
}
#endif
