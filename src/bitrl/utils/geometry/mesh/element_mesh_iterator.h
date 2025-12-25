#ifndef ELEMENT_ITERATOR_H
#define ELEMENT_ITERATOR_H

#include "bitrl/utils/filtered_iterator.h"

namespace bitrl
{
namespace utils
{
namespace geom
{

template <typename Predicate, typename MeshTp> class ElementMeshIterator
{

  public:
    typedef Predicate predicate_t;
    typedef MeshTp mesh_t;
    typedef typename mesh_t::element_iterator_impl iterator_impl;
    typedef typename FilteredIterator<Predicate, iterator_impl>::value_type value_type;
    typedef FilteredIterator<Predicate, iterator_impl> result_t;

    /// constructor
    ElementMeshIterator(mesh_t &mesh);

    /// \brief begin the iteration
    result_t begin();

    /// \brief end the iteration
    result_t end();

    /// \brief begin iteration
    template <typename T> result_t begin(const T &item);

    /// \brief end iteration
    template <typename T> result_t end(const T &item);

  protected:
    // the mesh over which the iterator is working
    mesh_t &mesh_;
};

template <typename Predicate, typename MeshTp>
ElementMeshIterator<Predicate, MeshTp>::ElementMeshIterator(
    typename ElementMeshIterator<Predicate, MeshTp>::mesh_t &mesh)
    : mesh_(mesh)
{
}

template <typename Predicate, typename MeshTp>
typename ElementMeshIterator<Predicate, MeshTp>::result_t
ElementMeshIterator<Predicate, MeshTp>::begin()
{
    Predicate p;
    return ElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_begin(),
                                                            mesh_.elements_end());
}

template <typename Predicate, typename MeshTp>
typename ElementMeshIterator<Predicate, MeshTp>::result_t
ElementMeshIterator<Predicate, MeshTp>::end()
{
    Predicate p;
    return ElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_end(),
                                                            mesh_.elements_end());
}

template <typename Predicate, typename MeshTp>
template <typename T>
typename ElementMeshIterator<Predicate, MeshTp>::result_t
ElementMeshIterator<Predicate, MeshTp>::begin(const T &item)
{
    Predicate p(item);
    return ElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_begin(),
                                                            mesh_.elements_end());
}

template <typename Predicate, typename MeshTp>
template <typename T>
typename ElementMeshIterator<Predicate, MeshTp>::result_t
ElementMeshIterator<Predicate, MeshTp>::end(const T &item)
{
    Predicate p(item);
    return ElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_end(),
                                                            mesh_.elements_end());
}

template <typename Predicate, typename MeshTp> class ConstElementMeshIterator
{

  public:
    typedef Predicate predicate_t;
    typedef MeshTp mesh_t;
    typedef typename mesh_t::celement_iterator_impl iterator_impl;
    typedef typename FilteredIterator<Predicate, iterator_impl>::value_type value_type;
    typedef FilteredIterator<Predicate, iterator_impl> result_t;

    /// constructor
    ConstElementMeshIterator(const mesh_t &mesh);

    /// \brief begin the iteration
    result_t begin() const;

    /// \brief end the iteration
    result_t end() const;

    /// \brief begin iteration
    template <typename T> result_t begin(const T &item) const;

    /// \brief end iteration
    template <typename T> result_t end(const T &item) const;

  protected:
    // the mesh over which the iterator is working
    const mesh_t &mesh_;
};

template <typename Predicate, typename MeshTp>
ConstElementMeshIterator<Predicate, MeshTp>::ConstElementMeshIterator(
    const typename ConstElementMeshIterator<Predicate, MeshTp>::mesh_t &mesh)
    : mesh_(mesh)
{
}

template <typename Predicate, typename MeshTp>
typename ConstElementMeshIterator<Predicate, MeshTp>::result_t
ConstElementMeshIterator<Predicate, MeshTp>::begin() const
{
    Predicate p;
    return ConstElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_begin(),
                                                                 mesh_.elements_end());
}

template <typename Predicate, typename MeshTp>
typename ConstElementMeshIterator<Predicate, MeshTp>::result_t
ConstElementMeshIterator<Predicate, MeshTp>::end() const
{
    Predicate p;
    return ConstElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_end(),
                                                                 mesh_.elements_end());
}

template <typename Predicate, typename MeshTp>
template <typename T>
typename ConstElementMeshIterator<Predicate, MeshTp>::result_t
ConstElementMeshIterator<Predicate, MeshTp>::begin(const T &item) const
{
    Predicate p(item);
    return ConstElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_begin(),
                                                                 mesh_.elements_end());
}

template <typename Predicate, typename MeshTp>
template <typename T>
typename ConstElementMeshIterator<Predicate, MeshTp>::result_t
ConstElementMeshIterator<Predicate, MeshTp>::end(const T &item) const
{
    Predicate p(item);
    return ConstElementMeshIterator<Predicate, MeshTp>::result_t(p, mesh_.elements_end(),
                                                                 mesh_.elements_end());
}

} // namespace geom

} // namespace utils
} // namespace bitrl

#endif // ELEMENT_ITERATOR_H
