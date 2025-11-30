//
// Created by alex on 11/23/25.
//

#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include "bitrl/bitrl_types.h"
#include <optional>
#include <string>

namespace bitrl
{
    namespace sensors
    {
        struct UltrasoundReading {
            real_t distance_m;
            real_t timestamp;
            static std::optional<UltrasoundReading> parse(const std::string& msg);
        };
    }
}


#endif //ULTRASOUND_H
