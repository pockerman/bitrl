#ifndef FILTERED_ITERATOR_H
#define FILTERED_ITERATOR_H

#include "boost/iterator/filter_iterator.hpp"

namespace bitrl
{

///
/// \brief Simple wrapper to boost::filter_iterator
///
template <typename PREDICATE, typename TYPE> class FilteredIterator
{

  public:
    typedef PREDICATE predicate;
    typedef TYPE iterator_type;
    typedef typename boost::filter_iterator<PREDICATE, TYPE> iterator_t;
    typedef typename iterator_t::value_type value_type;

    /// \brief default constructor
    FilteredIterator();

    /// \brief construct by passing in the predicate and the range
    /// over which we iterate
    FilteredIterator(predicate p, iterator_type begin, iterator_type end);

    /// \brief construct by passing in the predicate and the range
    /// over which we iterate
    FilteredIterator(iterator_type begin, iterator_type end);

    /// \brief copy cotor
    FilteredIterator(const FilteredIterator &o);

    /// \brief Destructor
    virtual ~FilteredIterator() {}

    /// \brief  op++ simply increment the boost::filter_iterator
    FilteredIterator &operator++();

    /// \brief  ++op simply increment the boost::filter_iterator
    FilteredIterator operator++(int);

    /// \brief Return the underlying boost iterator
    iterator_t get_iterator() { return it_; }

    /// \brief Dereference the iterator
    value_type operator*() const { return *it_; }

    template <typename Pred, typename Type>
    friend bool operator==(const FilteredIterator<Pred, Type> &lhs,
                           const FilteredIterator<Pred, Type> &rhs);

  private:
    /// \brief the boost::filter_iterator instance
    iterator_t it_;
};

template <typename PREDICATE, typename TYPE>
FilteredIterator<PREDICATE, TYPE>::FilteredIterator() : it_()
{
}

template <typename PREDICATE, typename TYPE>
FilteredIterator<PREDICATE, TYPE>::FilteredIterator(
    typename FilteredIterator<PREDICATE, TYPE>::predicate p,
    typename FilteredIterator<PREDICATE, TYPE>::iterator_type begin,
    typename FilteredIterator<PREDICATE, TYPE>::iterator_type end)
    : it_(boost::make_filter_iterator(p, begin, end))
{
}

template <typename PREDICATE, typename TYPE>
FilteredIterator<PREDICATE, TYPE>::FilteredIterator(
    typename FilteredIterator<PREDICATE, TYPE>::iterator_type begin,
    typename FilteredIterator<PREDICATE, TYPE>::iterator_type end)
    : it_(boost::make_filter_iterator(PREDICATE(), begin, end))
{
}

template <typename PREDICATE, typename TYPE>
FilteredIterator<PREDICATE, TYPE>::FilteredIterator(const FilteredIterator<PREDICATE, TYPE> &o)
    : it_(o.it_)
{
}

template <typename PREDICATE, typename TYPE>
FilteredIterator<PREDICATE, TYPE> &FilteredIterator<PREDICATE, TYPE>::operator++()
{
    it_++;
    return *this;
}

template <typename PREDICATE, typename TYPE>
FilteredIterator<PREDICATE, TYPE> FilteredIterator<PREDICATE, TYPE>::operator++(int)
{
    FilteredIterator<PREDICATE, TYPE> tmp(*this);
    ++it_;
    return tmp;
}

template <typename Pred, typename Type>
inline bool operator==(const FilteredIterator<Pred, Type> &lhs,
                       const FilteredIterator<Pred, Type> &rhs)
{
    return lhs.it_ == rhs.it_;
}

template <typename Pred, typename Type>
inline bool operator!=(const FilteredIterator<Pred, Type> &lhs,
                       const FilteredIterator<Pred, Type> &rhs)
{

    return !(lhs == rhs);
}

} // namespace bitrl
#endif
