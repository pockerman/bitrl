//
// Created by alex on 1/1/26.
//

#ifndef MESSAGE_BASE_H
#define MESSAGE_BASE_H

#include <chrono>
#include <string>
namespace bitrl
{
namespace sensors
{
struct MessageBase
{
    /**
     * Time stamp from the source that generated the message
     */
    std::chrono::system_clock::time_point source_timestamp;

    /**
     * The timestamp this message was constructed
     *
     */
    std::chrono::system_clock::time_point received_timestamp{std::chrono::system_clock::now()};

    /**
     * Convert the given time point to a Unix timestamp seconds since epoch
     * @param point
     * @return
     */
    static auto unix_seconds_since_epoch(std::chrono::system_clock::time_point point);

    /**
    * Convert the given time point to a Unix timestamp milliseconds since epoch
    * @param point
    * @return
    */
    static auto unix_milliseconds_since_epoch(std::chrono::system_clock::time_point point);

    /**
     * Convert the given string into a std::chrono::system_clock::time_point
     * The string is assumed that it represents time in UTC
     * @param ts
     * @return
     */
    static std::chrono::system_clock::time_point parse_utc_timestamp(const std::string& ts);
};
}
}

#endif //MESSAGE_BASE_H
