#ifndef GYMFCPP_CONSTS_H
#define GYMFCPP_CONSTS_H
/**
 * \file rlenvs_consts.h
 *
 */

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_config.h"
#include <numbers>
#include <string>

namespace bitrl
{
namespace consts
{

///
/// \brief INVALID_ID
///
inline const uint_t INVALID_ID = static_cast<uint_t>(-1);

///
/// \brief Invalid string
///
inline const std::string INVALID_STR = std::string("INVALID");

///
/// \brief Tolerance to use around the library
///
inline const real_t TOLERANCE = 1.0e-8;

inline const std::string ROBOTS_DIR = std::string(ROBOTS_DATA_DIR);


namespace maths
{

///
/// \brief The Pi constant
///
inline const real_t PI = std::numbers::pi;

///
/// \brief Acceleration due to gravity m/secs
///
inline const real_t G = 9.82;
} // namespace maths

} // namespace consts

} // namespace bitrl

#endif // GYMFCPP_CONSTS_H
