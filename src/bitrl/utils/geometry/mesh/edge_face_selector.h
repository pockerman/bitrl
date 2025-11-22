#ifndef EDGE_FACE_SELECTOR_H
#define EDGE_FACE_SELECTOR_H


namespace bitrl{
namespace utils{
namespace geom{

template<int spacedim> class Node;
template<int spacedim,int topodim> class FaceElement;


template<int dim,int topodim>
struct EdgeType;


template<int dim>
struct EdgeType<dim,0>
{


  typedef FaceElement<dim,0>*       ptr_t;
  typedef FaceElement<dim,0>&       ref_t;
  typedef const FaceElement<dim,0>* cptr_t;
  typedef const FaceElement<dim,0>& cref_t;


};
 
template<int dim>
struct EdgeType<dim,1>
{

  typedef FaceElement<dim,1>*       ptr_t;
  typedef FaceElement<dim,1>&       ref_t;
  typedef const FaceElement<dim,1>* cptr_t;
  typedef const FaceElement<dim,1>& cref_t;
};

template<int dim>
struct EdgeSelector
{

  typedef typename EdgeType<dim,1>::ptr_t  ptr_t;
  typedef typename EdgeType<dim,1>::ref_t  ref_t;
  typedef typename EdgeType<dim,1>::cptr_t   cptr_t;
  typedef typename EdgeType<dim,1>::cref_t   cref_t;

};
 
template<>
struct EdgeSelector<1>
{

  typedef EdgeType<1,0>::ptr_t  ptr_t;
  typedef EdgeType<1,0>::ref_t  ref_t;
  typedef EdgeType<1,0>::cptr_t cptr_t;
  typedef EdgeType<1,0>::cref_t cref_t;

};

}
 
}
}
#endif
