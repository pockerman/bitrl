#include "bitrl/bitrl_types.h"

///
/// \file math_utils.h
///

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace bitrl
{
namespace utils::maths
{

///
/// \fn sign
/// \brief Returns the sign of the expression.
/// The given expression should support the < comparison operator
///
template <typename T> int_t sign(const T &expr)
{

    if (expr < 0)
    {
        return -1;
    }

    return 1;
}

///
/// \fn sign
/// \brief Returns the sign when the two expressions are multipled
/// T should support the multiplication operation
///
template <typename T> int_t sign(const T &expr1, const T &expr2) { return sign(expr1 * expr2); }

///
/// \brief Compute the sum of squares of the elements of the given vector
/// \param The vector
/// \return sum of squares
template <typename T> T sum_sqr(const DynVec<T> &vec)
{

    T sum_ = T(0);
    for (uint_t i = 0; i < vec.size(); ++i)
    {
        sum_ += vec[i] * vec[i];
    }

    return sum_;
}

///
/// \fn sqr
/// \brief Returns the square of the given argument.
/// T should support the multiplication operation with itself
///
template <typename T> T sqr(const T &v) { return v * v; }

///
/// \brief Returns the index of the element that has the maximum
/// value in the array. Implementation taken from
/// http://www.jclay.host/dev-journal/simple_cpp_argmax_argmin.html
///
template <typename VectorType> uint_t arg_max(const VectorType &vec)
{
    return static_cast<uint_t>(
        std::distance(vec.begin(), std::max_element(vec.begin(), vec.end())));
}

///
/// \brief Returns the index of the element that has the minimum
/// value in the array. Implementation taken from
/// http://www.jclay.host/dev-journal/simple_cpp_argmax_argmin.html
///
template <typename VectorType> uint_t arg_min(const VectorType &vec)
{
    return static_cast<uint_t>(
        std::distance(vec.begin(), std::min_element(vec.begin(), vec.end())));
}

///
/// \brief Returns the max element in the vector
///
template <typename VectorType> typename VectorType::value_type max(const VectorType &vec)
{
    return *std::max_element(vec.begin(), vec.end());
}

///
/// \brief Returns the minimum element in the vector
///
template <typename VectorType> typename VectorType::value_type min(const VectorType &vec)
{
    return *std::min_element(vec.begin(), vec.end());
}

namespace
{

///
/// \brief Return a value in the linear sequence in the range
/// [start, stop]. The index parameter is the current number
/// in the sequence of the n total numbers we are about to
/// generate
///
template <typename T> auto lin_value(T start, T stop, uint_t index, uint_t n)
{

    assert(n > 1 && index < n);
    const auto amount = static_cast<T>(index) / (n - 1);
    const auto v = std::lerp(start, stop, amount);
    return v;
}

} // namespace

///
/// \brief Generate n evenly linearly spaced numbers between
/// [start, stop]. Analogous to NumPy linspace.
///
template <typename T> std::vector<T> lin_space(T start, T stop, uint_t n)
{

    auto v = std::vector<T>{};
    v.reserve(n);
    for (auto i = 0u; i < n; ++i)
    {
        v.push_back(lin_value(start, stop, i, n));
    }
    return v;
}

}
} // namespace bitrl
