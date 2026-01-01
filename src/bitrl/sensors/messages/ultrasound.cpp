#include "bitrl/sensors/messages/ultrasound.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

namespace bitrl
{
namespace sensors
{
std::optional<UltrasoundMessage> UltrasoundMessage::parse(const std::string &msg)
{
    auto json = nlohmann::json::parse(msg);
    auto timestamp = json["timestamp"];
    auto utc_timestamp = parse_utc_timestamp(timestamp);
    auto message = UltrasoundMessage();
    message.distance = json["distance"].get<real_t>();
    message.unit_str = json["unit"].get<std::string>();
    message.source_timestamp = utc_timestamp;
    return message;
}
} // namespace sensors
} // namespace bitrl
