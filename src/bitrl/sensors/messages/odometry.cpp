//
// Created by alex on 11/23/25.
//

#include "odometry.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

namespace bitrl
{
namespace sensors
{
std::optional<OdometryMessage> OdometryMessage::parse(const std::string &msg)
{
    auto json = nlohmann::json::parse(msg);
    return OdometryMessage{json["x"], json["y"], json["yaw"], json["timestamp"]};
}
} // namespace sensors
} // namespace bitrl