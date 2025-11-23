//
// Created by alex on 11/23/25.
//

#ifndef SENSOR_TYPE_ENUM_H
#define SENSOR_TYPE_ENUM_H
#include "bitrl/bitrl_types.h"

namespace bitrl
{
    namespace sensors
    {
        enum class SensorTypeEnum: uint_t {INVALID_TYPE=0, CAMERA=1, ULTRASOUND=2, ODMETRY=3, };
    }
}

#endif //SENSOR_TYPE_ENUM_H
