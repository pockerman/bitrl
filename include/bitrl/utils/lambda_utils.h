#ifndef LAMBDA_UTILS_H
#define LAMBDA_UTILS_H

#include "bitrl/base/cubeai_types.h"
#include <iostream>

namespace bitrl
{
namespace utils
{

///
/// \brief Utility lambda to print the given item.
/// Requires C++14
///
template <typename T> auto cubeai_print = [](const T &val) { std::cout << val << std::endl; };

} // namespace utils

} // namespace bitrl

#endif // ITERATION_COUNTER_H
