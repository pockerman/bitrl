#ifndef IO_UTILS_H
#define IO_UTILS_H

#include "bitrl/bitrl_types.h"
#include <ostream>
#include <vector>
#include <iomanip>
#include <chrono>

namespace bitrl
{

namespace utils::io
{
/**
* @param out The stream to write
* @param tp The time point to write on the stream
* @return Read/write reference to the stream we write on
*/
std::ostream& print_time_point(std::ostream &out,
                               const std::chrono::system_clock::time_point& tp);

template <typename T> std::ostream &print_vector(std::ostream &out, const std::vector<T> &obs)
{

    if (obs.empty())
    {
        out << "[]" << std::endl;
    }
    else
    {
        auto obs_str = std::to_string(obs[0]);
        for (uint_t i = 1; i < obs.size(); ++i)
        {
            obs_str += ",";
            obs_str += std::to_string(obs[i]);
        }
        out << obs_str << std::endl;
    }

    return out;
}

template <typename T>
std::ostream &print_vector(std::ostream &out, const std::vector<std::vector<T>> &obs)
{

    for (uint_t i = 0; i < obs.size(); ++i)
    {
        print_vector(out, obs[i]);
    }
    return out;
}
template <typename T>
std::ostream &print_vector(std::ostream &out, const std::vector<std::vector<std::vector<T>>> &obs)
{

    for (uint_t i = 0; i < obs.size(); ++i)
    {
        print_vector(out, obs[i]);
    }
    return out;
}

}

inline
std::ostream &operator<<(std::ostream &out, const std::chrono::system_clock::time_point &tp)
{
    return utils::io::print_time_point(out, tp);
}

/**
 *
 * @tparam T The type of the value to print
 * @param out The stream to write on
 * @param obs The values to print on out
 * @return Read/write reference to the stream
 */
template <typename T> std::ostream &operator<<(std::ostream &out,
                                               const std::vector<T> &obs)
{
    return utils::io::print_vector(out, obs);
}

} // namespace bitrl

#endif