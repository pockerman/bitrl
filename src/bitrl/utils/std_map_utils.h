#ifndef STD_MAP_UTILS_H
#define STD_MAP_UTILS_H

///
/// \file math_utils.h
///
/// Various utilities for std::map
///

#include <any>
#include <map>
#include <stdexcept>
#include <unordered_map>

namespace bitrl
{
namespace utils
{

/**
 * Given the name of the argument return std::any_cast<OutT>(itr->second)
 * where itr is itr = input.find(name)
 */
template <typename OutT>
OutT resolve(const std::string &name, const std::map<std::string, std::any> &input)
{

    auto itr = input.find(name);

    if (itr == input.end())
    {
        throw std::logic_error("Property: " + name + " not in input");
    }

    return std::any_cast<OutT>(itr->second);
}

/**
 * Given the name of the argument return std::any_cast<OutT>(itr->second)
 * where itr is itr = input.find(name)
 */
template <typename OutT>
OutT resolve(const std::string &name, const std::unordered_map<std::string, std::any> &input)
{
    auto itr = input.find(name);

    if (itr == input.end())
    {
        throw std::logic_error("Property: " + name + " not in input");
    }

    try
    {
        return std::any_cast<OutT>(itr->second);
    }
    catch (const std::bad_any_cast &)
    {
        throw std::logic_error("Property: " + name + " has unexpected type");
    }
}

} // namespace utils
} // namespace bitrl

#endif