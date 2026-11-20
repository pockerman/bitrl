#ifndef CUBEAI_CONCEPTS_H
#define CUBEAI_CONCEPTS_H
/**
 * Constraints used around the library to make code
 * safer and more readable. A nice introduction to
 * C++ concepts can be found at:
 *
 * https://www.youtube.com/watch?v=N_kPd2OK1L8
 * */

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH
#include "torch/torch.h"
#endif

#include <type_traits>

namespace bitrl
{
namespace utils
{
namespace concepts
{

template <typename VectorType>
concept integral_vector = std::is_integral<typename VectorType::value_type>::value;

template <typename VectorType>
concept float_vector = std::is_floating_point<typename VectorType::value_type>::value;

template <typename VectorType>
concept float_or_integral_vector = std::is_integral<typename VectorType::value_type>::value ||
    std::is_floating_point<typename VectorType::value_type>::value;

template <typename Type>
concept is_default_constructible = std::is_default_constructible<Type>::value;

#ifdef USE_PYTORCH
template <typename AlgoType>
concept pytorch_module = std::is_base_of<torch::nn::Module, AlgoType>::value;
#endif

} // namespace concepts

} // namespace utils
} // namespace bitrl

#endif // CUBEAI_CONSTRAINTS_H
