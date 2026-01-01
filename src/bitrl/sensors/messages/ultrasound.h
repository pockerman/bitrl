//
// Created by alex on 11/23/25.
//

#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/messages/message_base.h"
#include <optional>
#include <string>

namespace bitrl
{
namespace sensors
{

/**
 * struct UltrasoundMessage. Models a message from an Ultrasound sensor
 */
struct UltrasoundMessage: MessageBase
{
    real_t distance;
    std::string unit_str;
    static std::optional<UltrasoundMessage> parse(const std::string &msg);
};
} // namespace sensors
} // namespace bitrl

#endif // ULTRASOUND_H
