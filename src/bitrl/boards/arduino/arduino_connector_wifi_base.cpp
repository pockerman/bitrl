//
// Created by alex on 8/2/25.
//

#include "bitrl/boards/arduino/arduino_connector_wifi_base.h"
#include "bitrl/extern/HTTPRequest.hpp"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <string>

namespace bitrl
{
namespace boards::arduino
{
ArduinoConnectorWIFIBase::ArduinoConnectorWIFIBase(const std::string &arduino_url)
    : ArduinoConnectorBase(), arduino_url_(arduino_url)
{
}

void ArduinoConnectorWIFIBase::connect() {}

void ArduinoConnectorWIFIBase::close_connection() {}

std::string ArduinoConnectorWIFIBase::send_cmd(const ArduinoCMDBase &cmd)
{
    http::Request request{arduino_url_};
    std::string body_str = cmd.get_cmd();

    const http::Response response = request.send("POST", body_str);
    auto str_response = std::string(response.body.begin(), response.body.end());
    return str_response;
}

} // namespace boards::arduino
} // namespace bitrl
