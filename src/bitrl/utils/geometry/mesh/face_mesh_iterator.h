#ifndef FACE_MESH_ITERATOR_H
#define FACE_MESH_ITERATOR_H

#include "bitrl/utils/filtered_iterator.h"

namespace bitrl{
namespace utils{
namespace geom{

template<typename Predicate, typename  MeshTp> class FaceMeshIterator;

/// \brief The general template
template<typename Predicate, typename  MeshTp>
class FaceMeshIterator
{
public:

    typedef Predicate predicate_t;
    typedef MeshTp mesh_t;
    typedef typename mesh_t::face_iterator_impl iterator_impl;
    typedef typename FilteredIterator<Predicate, iterator_impl>::value_type value_type;
    typedef FilteredIterator<Predicate, iterator_impl> result_t;

    /// constructor
    FaceMeshIterator(mesh_t& mesh);

    /// \brief begin the iteration
    result_t begin();

    /// \brief end the iteration
    result_t end();

protected:

    // the mesh over which the iterator is working
    mesh_t& mesh_;

};

template<typename Predicate, typename  MeshTp>
FaceMeshIterator<Predicate, MeshTp>::FaceMeshIterator(typename FaceMeshIterator<Predicate, MeshTp>::mesh_t& mesh)
    :
mesh_(mesh)
{}

template<typename Predicate, typename  MeshTp>
typename FaceMeshIterator<Predicate, MeshTp>::result_t
FaceMeshIterator<Predicate, MeshTp>::begin(){
    Predicate p;
    return FaceMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.faces_begin(), mesh_.faces_end() );
}

template<typename Predicate, typename  MeshTp>
typename FaceMeshIterator<Predicate, MeshTp>::result_t
FaceMeshIterator<Predicate, MeshTp>::end(){
    Predicate p;
    return FaceMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.faces_end(), mesh_.faces_end() );
}


/// \brief The general template
template<typename Predicate, typename  MeshTp>
class ConstFaceMeshIterator
{
public:

    typedef Predicate predicate_t;
    typedef MeshTp mesh_t;
    typedef typename mesh_t::cface_iterator_impl iterator_impl;
    typedef typename FilteredIterator<Predicate, iterator_impl>::value_type value_type;
    typedef FilteredIterator<Predicate, iterator_impl> result_t;

    /// constructor
    ConstFaceMeshIterator(const mesh_t& mesh);

    /// \brief begin the iteration
    result_t begin()const;

    /// \brief end the iteration
    result_t end()const;

protected:

    // the mesh over which the iterator is working
    const mesh_t& mesh_;

};

template<typename Predicate, typename  MeshTp>
ConstFaceMeshIterator<Predicate, MeshTp>::ConstFaceMeshIterator(const typename ConstFaceMeshIterator<Predicate, MeshTp>::mesh_t& mesh)
    :
mesh_(mesh)
{}

template<typename Predicate, typename  MeshTp>
typename ConstFaceMeshIterator<Predicate, MeshTp>::result_t
ConstFaceMeshIterator<Predicate, MeshTp>::begin()const{
    Predicate p;
    return ConstFaceMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.faces_begin(), mesh_.faces_end() );
}

template<typename Predicate, typename  MeshTp>
typename ConstFaceMeshIterator<Predicate, MeshTp>::result_t
ConstFaceMeshIterator<Predicate, MeshTp>::end()const{
    Predicate p;
    return ConstFaceMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.faces_end(), mesh_.faces_end() );
}

}
}
}

#endif // FACE_MESH_ITERATOR_H
