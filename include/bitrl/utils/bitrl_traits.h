#ifndef CUBEAI_TRAITS_H
#define CUBEAI_TRAITS_H

#include "cubeai/base/cubeai_types.h"

#include <vector>

namespace bitrl
{
namespace utils
{

template <typename VecType> struct vector_value_type_trait;

template <typename T> struct vector_value_type_trait<std::vector<T>>
{
    typedef typename std::vector<T>::value_type value_type;
};

template <typename T> struct vector_value_type_trait<DynVec<T>>
{
    typedef typename std::vector<T>::value_type value_type;
};

template <typename T> struct vector_value_type_trait<std::pair<std::vector<T>, uint_t>>
{
    typedef typename std::vector<T>::value_type value_type;
};

template <typename T> struct vector_value_type_trait<std::pair<DynVec<T>, uint_t>>
{
    typedef typename std::vector<T>::value_type value_type;
};

} // namespace utils
} // namespace bitrl

#endif // CUBEAI_TRAITS_H
