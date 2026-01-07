#include "bitrl/sensors/messages/message_base.h"
#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace bitrl
{
namespace sensors
{

    auto MessageBase::unix_seconds_since_epoch(std::chrono::system_clock::time_point point)
    {
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
            point.time_since_epoch()
        ).count();

    }


    auto MessageBase::unix_milliseconds_since_epoch(std::chrono::system_clock::time_point point)
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(point.time_since_epoch()).count();
    }

    std::chrono::system_clock::time_point MessageBase::parse_utc_timestamp(const std::string& ts)
    {
        std::tm tm{};
        std::istringstream ss(ts);

        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
        if (ss.fail())
            throw std::runtime_error("Invalid timestamp format");

        // Convert tm (UTC) → time_t (UTC)
        std::time_t t = timegm(&tm);

        return std::chrono::system_clock::from_time_t(t);
    }

}
}
