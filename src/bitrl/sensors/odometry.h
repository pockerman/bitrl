//
// Created by alex on 11/23/25.
//

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "bitrl/bitrl_types.h"
#include <optional>
#include <string>

namespace bitrl
{
namespace sensors
{
struct OdometryReading
{
    real_t x;
    real_t y;
    real_t yaw;
    real_t timestamp;

    static std::optional<OdometryReading> parse(const std::string &msg);
};
} // namespace sensors
} // namespace bitrl

#endif // ODOMETRY_H
