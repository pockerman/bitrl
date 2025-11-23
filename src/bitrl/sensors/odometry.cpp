//
// Created by alex on 11/23/25.
//

#include "bitrl/sensors/odometry.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

namespace bitrl
{
    namespace sensors
    {
            std::optional<OdometryReading>
            OdometryReading::parse(const std::string& msg) {
                auto json = nlohmann::json::parse(msg);
                return OdometryReading{
                    json["x"], json["y"], json["yaw"], json["timestamp"]
                };
            }
    }
}