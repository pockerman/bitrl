#ifndef CUBEAI_TYPES_H
#define CUBEAI_TYPES_H

#include "bitrl/base/bitrl_config.h"
#include "bitsim/bitsim_types.h"

#ifdef USE_PYTORCH
#include <torch/torch.h>
#endif

#include <cstddef>
#include <string>

namespace bitrl
{

using namespace bitsim;

///
/// \brief int type
///
using bitsim::int_t;

///
/// \brief long int type
///
using bitsim::lint_t;

///
/// \brief The float precision type
///
using bitsim::float_t;

///
/// \brief The double precision type
///
using bitsim::real_t;

///
/// \brief General matrix type
///
using bitsim::DynMat;

///
/// \brief Square matrix type
///
using bitsim::SquareMat;

///
/// General fixed size at compile time matrix
///
using bitsim::Mat;

///
/// \brief General row vector
///
using bitsim::DynVec;

///
/// Float type vector
///
using bitsim::FloatVec;

///
/// Real type vector
///
using bitsim::RealVec;

///
/// \brief Null type. Simple placeholder
///
using bitsim::Null;

///
/// \brief Device type
///
using bitsim::DeviceType;

#ifdef USE_PYTORCH
///
/// \brief placeholder for torch
///
struct torch_t
{
};

///
/// \brief torch_int_t
///
typedef long int torch_int_t;

///
/// \brief torch_tensor_t
///
typedef torch::Tensor torch_tensor_t;

template <typename T> struct torch_tensor_value_type
{
    typedef T value_type;
};
#endif

} // namespace bitrl

#endif
