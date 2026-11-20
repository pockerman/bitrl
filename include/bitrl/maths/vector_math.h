#ifndef BASIC_ARRAY_STATISTICS_H
#define BASIC_ARRAY_STATISTICS_H
/**
 * Implements basic statistical computations on arrays
 */

#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/utils/bitrl_concepts.h"
#include "bitsim/bitsim_consts.h"

#include <algorithm>
#include <cmath>
#include <execution>
#include <iostream>
#include <iterator>
#include <random>
#include <stdexcept>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl
{
namespace maths
{

namespace detail_
{

template <typename T> struct randomize_handler;

template <> struct randomize_handler<real_t>
{
    template <typename VecType> static void randomize_(VecType &v, real_t a, real_t b, uint_t seed);
};

template <typename VecType>
void randomize_handler<real_t>::randomize_(VecType &v, real_t a, real_t b, uint_t seed)
{

    std::mt19937 generator(seed);
    std::uniform_real_distribution<real_t> distribution(a, b);

    std::for_each(v.begin(), v.end(), [&](auto &val) { val = distribution(generator); });
}

template <> struct randomize_handler<float_t>
{
    template <typename VecType>
    static void randomize_(VecType &v, float_t a, float_t b, uint_t seed);
};

template <typename VecType>
void randomize_handler<float_t>::randomize_(VecType &v, float_t a, float_t b, uint_t seed)
{

    std::mt19937 generator(seed);
    std::uniform_real_distribution<float_t> distribution(a, b);

    std::for_each(v.begin(), v.end(), [&](auto &val) { val = distribution(generator); });
}
} // namespace detail_

template <utils::concepts::float_or_integral_vector VectorType>
VectorType element_product(const VectorType &v1, const VectorType &v2)
{

    auto size1_ = v1.size();
    auto size2_ = v2.size();

    if (size1_ != size2_)
    {
        throw std::logic_error(
            "Sizes not equal. Cannot compute element product of vectors with no equal sizes");
    }

    VectorType v_(v1.size(), 0);
    for (uint_t i = 0; i < v1.size(); ++i)
    {

        v_[i] = v1[i] * v2[i];
    }
    return v_;
}

template <typename IteratorType>
typename std::iterator_traits<IteratorType>::value_type sum(IteratorType begin, IteratorType end,
                                                            bool parallel = true)
{

    typedef typename std::iterator_traits<IteratorType>::value_type value_type;
    value_type sum_ = value_type(0);

    if (parallel)
    {
        sum_ = std::reduce(std::execution::par, begin, end, sum_);
    }
    else
    {
        sum_ = std::accumulate(begin, end, sum_);
    }

    return sum_;
}

template <typename VectorType>
typename VectorType::value_type sum(const VectorType &vec, bool parallel = true)
{
    return sum(vec.begin(), vec.end(), parallel);
}

///
/// \brief mean Compute the mean value of the values in
/// the provided iterator range
///
template <typename IteratorType>
real_t mean(IteratorType begin, IteratorType end, bool parallel = true)
{

    auto sum_ = sum(begin, end, parallel);
    return sum_ / static_cast<real_t>(std::distance(begin, end));
}

///
/// \brief mean Computes the mean value of the vector. If parallel=true
/// it uses std::reduce
///
template <utils::concepts::float_or_integral_vector VectorType>
real_t mean(const VectorType &vector, bool parallel = true)
{
    return mean(vector.begin(), vector.end(), parallel);
}

///
/// \brief mean computes the mean value of the given DynVec
///
template <typename T> real_t mean(const DynVec<T> &vector, bool parallel = true)
{
    return mean(vector.begin(), vector.end(), parallel);
}

template <typename IteratorType>
typename std::iterator_traits<IteratorType>::value_type
variance(IteratorType begin, IteratorType end, bool parallel = true)
{

    typedef typename std::iterator_traits<IteratorType>::value_type value_type;
    auto mean_val = mean(begin, end, parallel);
    auto size = std::distance(begin, end);

    std::vector<value_type> diff(size);
    std::transform(begin, end, diff.begin(), [mean_val](value_type x) { return x - mean_val; });

    value_type sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    return sq_sum / static_cast<value_type>(std::distance(begin, end));
}

///
/// \brief Compute the variance of the given std::vector
/// of real values
///
inline real_t variance(const std::vector<real_t> &vals, bool parallel = true)
{
    return variance(vals.begin(), vals.end(), parallel);
}

///
/// \brief Standardize the given vector
///
std::vector<real_t> standardize(const std::vector<real_t> &vals,
                                real_t tol = bitsim::consts::TOLERANCE);

///
/// \brief choice. Implements similar functionality
/// to numpy.choice function
///
/// https://www.cplusplus.com/reference/random/discrete_distribution/
///
template <utils::concepts::float_vector Vec2> uint_t choice(const Vec2 &probs, uint_t seed = 42)
{

    std::mt19937 generator(seed);
    std::discrete_distribution<int> distribution(probs.begin(), probs.end());
    return distribution(generator);
}

///
/// \brief choice. Implements similar functionality
/// to numpy.choice function
///
/// https://www.cplusplus.com/reference/random/discrete_distribution/
///
template <utils::concepts::integral_vector Vec1, utils::concepts::float_vector Vec2>
uint_t choice(const Vec1 &choices, const Vec2 &probs, uint_t seed = 42)
{

    std::mt19937 generator(seed);
    std::discrete_distribution<int> distribution(probs.begin(), probs.end());
    return choices[distribution(generator)];
}

template <utils::concepts::float_vector Vec>
typename Vec::value_type choose_value(const Vec &vals, uint_t seed = 42)
{

    std::mt19937 generator(seed);
    auto size = std::distance(vals.begin(), vals.end());
    std::discrete_distribution<int> distribution(0, static_cast<int>(size));
    auto rand_idx = distribution(generator);
    return vals[rand_idx];
}

///
/// \brief Given a vector of intergal or floating point values
/// and a set of values to choose from, randomize the entries ofv
///
template <utils::concepts::float_or_integral_vector Vec>
void randomize_vec(Vec &v, const Vec &walk_set, uint_t seed = 42)
{
    std::for_each(v.begin(), v.end(), [&](auto &val) { val += choose_value(walk_set, seed); });
}

///
/// \brief Fill in the given vector with random values
/// \param vec The vector to fill in
/// \param seed The seed for the random engine
///
template <typename T> std::vector<T> &randomize(std::vector<T> &vec, T a, T b, uint_t seed = 42)
{
    detail_::randomize_handler<T>::randomize_(vec, a, b, seed);
    return vec;
}

template <utils::concepts::float_or_integral_vector Vec>
Vec &divide(Vec &v1, typename Vec::value_type val)
{

    // both vectors must have the same size
    for (uint_t i = 0; i < v1.size(); ++i)
    {
        v1[i] /= val;
    }
    return v1;
}

template <utils::concepts::float_or_integral_vector Vec> Vec &add(Vec &v1, const Vec &v2)
{

    // both vectors must have the same size
    for (uint_t i = 0; i < v1.size(); ++i)
    {
        v1[i] += v2[i];
    }
    return v1;
}

template <utils::concepts::float_or_integral_vector VectorType>
VectorType exponentiate(const VectorType &vec)
{
    VectorType vec_exp(vec.size());
    uint_t counter = 0;
    static auto func = [&counter](const auto &data) { return std::pow(data, counter++); };
    std::transform(vec.begin(), vec.end(), vec_exp.begin(), func);
    return vec_exp;
}

template <utils::concepts::float_or_integral_vector VectorType>
VectorType exponentiate(const VectorType &vec, typename VectorType::value_type v)
{
    VectorType vec_exp(vec.size());
    uint_t counter = 0;
    static auto func = [&counter, v](const auto &) { return std::pow(v, counter++); };
    std::transform(vec.begin(), vec.end(), vec_exp.begin(), func);
    return vec_exp;
}

#ifdef USE_PYTORCH
template <utils::concepts::float_or_integral_vector VectorType>
VectorType exponentiate(const torch_tensor_t tensor, typename VectorType::value_type v)
{

#ifdef BITRL_DEBUG
    assert(tensor.dim() == 1 && "Invalid tensor dimension. Should be 1");
#endif

    VectorType vec_exp(tensor.size(0));
    uint_t counter = 0;

    for (uint_t i = 0; i < static_cast<uint_t>(tensor.size(0)); ++i)
    {

        auto val = tensor[i].item();
        vec_exp[i] = val.template to<typename VectorType::value_type>();
    }

    static auto func = [&counter, v](auto &val)
    {
        auto expo = std::pow(v, counter++);
        return val * expo;
    };

    std::for_each(vec_exp.begin(), vec_exp.end(), func);
    return vec_exp;
}
#endif

///
/// \brief applies softmax operation to the elements of the vector
/// and returns a vector with the result
///
template <typename T> std::vector<T> softmax_vec(const std::vector<T> &vec, real_t tau = 1.0)
{

    std::vector<T> result(vec.size());
    auto exp_sum = 0.0;
    auto vec_size = static_cast<uint_t>(vec.size());
    // calculate the exponentials
    for (uint_t i = 0; i < vec_size; ++i)
    {
        result[i] = std::exp(vec[i] / tau);
        exp_sum += result[i];
    }

    for (uint_t i = 0; i < vec_size; ++i)
    {
        result[i] /= exp_sum;
    }

    return result;
}

///
/// \brief applies softmax operation to the elements of the vector
/// and returns a vector with the result
///
template <typename T> DynVec<T> softmax_vec(const DynVec<T> &vec, real_t tau = 1.0)
{

    DynVec<T> result(vec.size());
    auto exp_sum = 0.0;
    auto vec_size = static_cast<uint_t>(vec.size());
    // calculate the exponentials
    for (uint_t i = 0; i < vec_size; ++i)
    {
        result[i] = std::exp(vec[i] / tau);
        exp_sum += result[i];
    }

    result /= exp_sum;
    return result;
}

///
/// \brief applies softmax operation to the elements of the vector
/// and returns a vector with the result
///
template <typename IteratorType>
DynVec<typename IteratorType::value_type> softmax_vec(IteratorType begin, IteratorType end,
                                                      real_t tau = 1.0)
{
    auto vec_size = std::distance(begin, end);
    DynVec<typename IteratorType::value_type> result(vec_size);
    auto exp_sum = 0.0;

    auto counter = 0;
    for (; begin != end; ++begin)
    {
        result[counter] = std::exp(*begin / tau);
        exp_sum += result[counter++];
    }

    result /= exp_sum;
    return result;
}

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

template <typename T> std::vector<uint_t> max_indices(const DynVec<T> &vec)
{

    // find max value
    auto max_val = vec.maxCoeff();

    auto result = std::vector<uint_t>();

    for (uint_t i = 0; i < static_cast<uint_t>(vec.size()); ++i)
    {
        T value = vec[i];
        if (value == max_val)
        {
            result.push_back(i);
        }
    }

    return result;
}

///
/// \brief Returns the indices of vec
/// where the maximum value in vec occurs
///
template <typename VecTp> std::vector<uint_t> max_indices(const VecTp &vec)
{

    // find max value
    auto max_val = std::max_element(vec.begin(), vec.end());

    auto result = std::vector<uint_t>();
    auto counter = 0;

    std::for_each(vec.begin(), vec.end(),
                  [&](auto val)
                  {
                      if (val == max_val)
                      {
                          result.push_back(counter);
                      }
                      ++counter;
                  });

    return result;
}

template <typename T>
std::vector<T> extract_subvector(const std::vector<T> &vec, uint_t end, bool up_to = true)
{

#ifdef BITRL_DEBUG
    assert(end <= vec.size() && "Invalid end index");
#endif

    if (up_to)
    {
        return std::vector<real_t>(vec.begin(), vec.begin() + end);
    }

    return std::vector<real_t>(vec.begin() + end, vec.end());
}

///
/// \brief bin_index. Compute sequnce[i - 1] <= x sequnce[i] and returns the
/// index. Sequence should be sorted
///
template <typename SequenceTp>
uint_t bin_index(const typename SequenceTp::value_type &x, const SequenceTp &sequence)
{

    if (sequence.size() <= 1)
    {
        return bitsim::consts::INVALID_ID;
    }

    auto index = 1;
    auto begin = sequence.begin();
    auto prev_val = *begin;
    ++begin;
    auto end = sequence.end();

    for (; begin != end; ++begin, ++index)
    {

        auto current_val = *begin;

        if ((prev_val <= x) && (x < current_val))
        {
            return index;
        }

        prev_val = current_val;
    }

    return bitsim::consts::INVALID_ID;
}

///
///@brief zero_center. Subtracts the mean value of the
/// given vector from every value of the vector
///
template <typename VectorType> VectorType zero_center(const VectorType &vec, bool parallel = true)
{

    auto vec_mean = mean(vec.begin(), vec.end(), parallel);
    VectorType v(vec.size());
    std::transform(vec.begin(), vec.end(), v.begin(),
                   [vec_mean](auto val) { return val - vec_mean; });
    return v;
}

///
/// \brief Normalize the values of the given vector with the given value.
/// Essentially this function divides the elements of the vector with the
/// value provided
/// \param vec
/// \param v
/// \param parallel
///
template <typename VectorType>
VectorType normalize(const VectorType &vec, typename VectorType::value_type v)
{

    VectorType v_(vec.size(), 0);

    for (uint_t i = 0; i < v_.size(); ++i)
    {
        v_[i] = vec[i] / v;
    }
    return v_;
}

template <typename T> std::vector<T> normalize_max(const std::vector<T> &vec)
{

    auto max_val = std::max_element(vec.begin(), vec.end());
    std::vector<T> v_(vec.size(), T(0));

    for (uint_t i = 0; i < v_.size(); ++i)
    {
        v_[i] = vec[i] / *max_val;
    }
    return v_;
}

template <typename T> std::vector<T> normalize_min(const std::vector<T> &vec)
{

    auto min_val = std::min_element(vec.begin(), vec.end());
    std::vector<T> v_(vec.size(), T(0));

    for (uint_t i = 0; i < v_.size(); ++i)
    {
        v_[i] = vec[i] / *min_val;
    }
    return v_;
}

/**
 * Return numbers spaced evenly on a log scale.
 * The implementation is inspired from numpy:
 * https://numpy.org/doc/stable/reference/generated/numpy.logspace.html See also:
 * https://quick-bench.com/q/Hs39BWQf5kr5Gjnv6zQkLXMrsDw
 *
 * The starting point of the scale is: std::pow(base, start)
 * Similarly the endpoint is: std::pow(base, end)
 * The intermediate points are aligned as  std::pow(base, point)
 * where point = start + i*dx
 * where
 * dx = (end - start) / (num - 1)
 *
 *
 * */
std::vector<real_t> logspace(real_t start, real_t end, uint_t num, real_t base = 10.0);

template <typename IteratorType>
typename std::iterator_traits<IteratorType>::value_type
dot_product(IteratorType bv1, IteratorType ev1, IteratorType bv2, IteratorType ev2)
{

    typedef typename std::iterator_traits<IteratorType>::value_type value_type;

    auto size1_ = std::distance(bv1, ev1);
    auto size2_ = std::distance(bv2, ev2);

    if (size1_ != size2_)
    {
        throw std::logic_error(
            "Sizes not equal. Cannot compute dot product of vectors with no equal sizes");
    }

    value_type sum_ = value_type(0);

    for (; bv1 != ev1; ++bv1, ++bv2)
    {
        sum_ += (*bv1) * (*bv2);
    }

    return sum_;
}

template <typename VectorType>
real_t dot_product(const VectorType &v1, const VectorType &v2, uint_t start_idx = 0)
{

#ifdef BITRL_DEBUG
    assert(v1.size() == v2.size() && "Invalid vector sizes");
    assert(start_idx < v1.size() && "Invalid start_idx");
#endif

    auto v1_begin = v1.begin();
    std::advance(v1_begin, start_idx);

    auto v2_begin = v2.begin();
    std::advance(v2_begin, start_idx);
    real_t dot_product = 0.0;

    for (; v1_begin != v1.end(); ++v1_begin, ++v2_begin)
    {
        dot_product += (*v1_begin) * (*v2_begin);
    }
    return dot_product;
}
} // namespace maths
} // namespace bitrl

#endif // BASIC_ARRAY_STATISTICS_H
