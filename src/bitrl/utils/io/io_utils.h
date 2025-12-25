#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <ostream>
#include <vector>

namespace bitrl
{
namespace utils::io
{

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

template <typename T> std::ostream &operator<<(std::ostream &out, const std::vector<T> &obs)
{
    return print_vector(out, obs);
}

} // namespace utils::io
} // namespace bitrl

#endif