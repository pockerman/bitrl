#ifndef ELEMENT_TRAITS_H
#define ELEMENT_TRAITS_H

#include "bitrl/utils/geometry/mesh/edge_face_selector.h"

namespace bitrl{
namespace utils{
namespace geom{

  template<int dim> class Element;
  template<int dim, int topodim> class FaceElement;

  template<typename ELEMENT_TYPE>
  struct element_traits;
  
  template<>
  struct element_traits<Element<1> >
  {
  
    static const int dim_ = 1;
    static const int topodim_ = 1;
    
    typedef EdgeSelector<1>::ptr_t      edge_ptr_t;
    typedef EdgeSelector<1>::ref_t      edge_ref_t;
    typedef EdgeSelector<1>::cptr_t     cedge_ptr_t;
    typedef EdgeSelector<1>::cref_t     cedge_ref_t;
    
    typedef EdgeSelector<1>::ptr_t      face_ptr_t;
    typedef EdgeSelector<1>::ref_t      face_ref_t;
    typedef EdgeSelector<1>::cptr_t     cface_ptr_t;
    typedef EdgeSelector<1>::cref_t     cface_ref_t;
  
  };
  
  template<>
  struct element_traits<Element<2> >
  {
  
    static const int dim_ = 2;
    static const int topodim_ = 2;
    
    typedef EdgeSelector<2>::ptr_t   edge_ptr_t;
    typedef EdgeSelector<2>::ref_t   edge_ref_t;
    typedef EdgeSelector<2>::cptr_t  cedge_ptr_t;
    typedef EdgeSelector<2>::cref_t  cedge_ref_t;

    typedef EdgeSelector<2>::ptr_t   face_ptr_t;
    typedef EdgeSelector<2>::ref_t   face_ref_t;
    typedef EdgeSelector<2>::cptr_t  cface_ptr_t;
    typedef EdgeSelector<2>::cref_t  cface_ref_t;
  
  };
  
  template<>
  struct element_traits<Element<3> >
  {
  
    static const int dim_ = 3;
    static const int topodim_ = 3;
    
    typedef EdgeSelector<3>::ptr_t   edge_ptr_t;
    typedef EdgeSelector<3>::ref_t   edge_ref_t;
    typedef EdgeSelector<3>::cptr_t  cedge_ptr_t;
    typedef EdgeSelector<3>::cref_t  cedge_ref_t;
    
    typedef FaceElement<3,2>*        face_ptr_t;
    typedef FaceElement<3,2>&        face_ref_t;
    typedef const FaceElement<3,2>*  cface_ptr_t;
    typedef const FaceElement<3,2>&  cface_ref_t;
  
  };
  
  ///
  /// \brief FaceElement<2,1> is an Edge in 2D. Edges always
  ///have Node objects as their terminal edges and faces
  ///
  template<>
  struct element_traits<FaceElement<2,1> >
  {
  
    static const int dim_ = 2;
    static const int topodim_ = 1;
    
    typedef Node<2>* edge_ptr_t;
    typedef Node<2>& edge_ref_t;
    typedef const Node<2>* cedge_ptr_t;
    typedef const Node<2>& cedge_ref_t;
    
    typedef Node<2>* face_ptr_t;
    typedef Node<2>& face_ref_t;
    typedef const Node<2>* cface_ptr_t;
    typedef const Node<2>& cface_ref_t;
    
  };
  
  /**
    * FaceElement<3,1> is an Edge in 3D. Edges always
    * have Node objects as their terminal edges and faces
    */
  template<>
  struct element_traits<FaceElement<3,1> >
  {
  
    static const int dim_ = 3;
    static const int topodim_ = 1;
    
    typedef Node<3>* edge_ptr_t;
    typedef Node<3>& edge_ref_t;
    typedef const Node<3>* cedge_ptr_t;
    typedef const Node<3>& cedge_ref_t;
    
    typedef Node<3>* face_ptr_t;
    typedef Node<3>& face_ref_t;
    typedef const Node<3>* cface_ptr_t;
    typedef const Node<3>& cface_ref_t;
    
  };
  
  
  template<>
  struct element_traits<FaceElement<3,2> >
  {
  
    static const int dim_ = 3;
    static const int topodim_ = 2;
    
    typedef EdgeSelector<3>::ptr_t      edge_ptr_t;
    typedef EdgeSelector<3>::ref_t      edge_ref_t;
    typedef EdgeSelector<3>::cptr_t cedge_ptr_t;
    typedef EdgeSelector<3>::cref_t cedge_ref_t;
    
    typedef EdgeSelector<3>::ptr_t      face_ptr_t;
    typedef EdgeSelector<3>::ref_t      face_ref_t;
    typedef EdgeSelector<3>::cptr_t cface_ptr_t;
    typedef EdgeSelector<3>::cref_t cface_ref_t;
  
  };

}
}
}
#endif
