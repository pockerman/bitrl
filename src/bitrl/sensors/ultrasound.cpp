//
// Created by alex on 11/23/25.
//

#include "bitrl/sensors/ultrasound.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

namespace bitrl
{
    namespace sensors
    {
            std::optional<UltrasoundReading>
            UltrasoundReading::parse(const std::string& msg)
            {
                auto json = nlohmann::json::parse(msg);
                return UltrasoundReading{
                    json["distance_m"], json["timestamp"]
                };
            }
    }
}
